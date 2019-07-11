package org.mavlink.qgroundcontrol;

/* Copyright 2013 Google Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * Project home page: http://code.google.com/p/usb-serial-for-android/
 */
///////////////////////////////////////////////////////////////////////////////////////////
//  Written by: Mike Goza April 2014
//
//  These routines interface with the Android USB Host devices for serial port communication.
//  The code uses the usb-serial-for-android software library.  The QGCActivity class is the
//  interface to the C++ routines through jni calls.  Do not change the functions without also
//  changing the corresponding calls in the C++ routines or you will break the interface.
//
////////////////////////////////////////////////////////////////////////////////////////////

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Method;
import java.lang.reflect.Field;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.*;
import android.widget.Toast;
import android.util.Log;
import android.os.PowerManager;
import android.view.WindowManager;
import android.os.Bundle;
import android.view.KeyEvent;
import android.net.wifi.WifiManager;
import android.net.wifi.WifiConfiguration;
import android.net.ConnectivityManager;
import android.os.Handler;
import android.os.ResultReceiver;
import android.os.storage.StorageManager;
import android.os.storage.StorageVolume;
import android.media.MediaScannerConnection;

import com.hoho.android.usbserial.driver.*;
import org.qtproject.qt5.android.bindings.QtActivity;
import org.qtproject.qt5.android.bindings.QtApplication;

public class QGCActivity extends QtActivity
{
    public  static int BAD_PORT = 0;
    private static QGCActivity m_instance;
    private static UsbManager m_manager;    //  ANDROID USB HOST CLASS
    private static List<UsbSerialDriver> m_devices; //  LIST OF CURRENT DEVICES
    private static HashMap<Integer, UsbSerialDriver> m_openedDevices;   //  LIST OF OPENED DEVICES
    private static HashMap<Integer, UsbIoManager> m_ioManager;	//  THREADS FOR LISTENING FOR INCOMING DATA
    private static HashMap<Integer, Integer> m_userData;    //  CORRESPONDING USER DATA FOR OPENED DEVICES.  USED IN DISCONNECT CALLBACK
    //  USED TO DETECT WHEN A DEVICE HAS BEEN UNPLUGGED
    private BroadcastReceiver m_UsbReceiver = null;
    private final static ExecutorService m_Executor = Executors.newSingleThreadExecutor();
    private static final String TAG = "QGC_QGCActivity";
    private static PowerManager.WakeLock m_wl;
    private static WifiManager m_wifiManager;
    private static WifiConfiguration m_wifiConfig;
    private static ConnectivityManager m_Cm;
    private static Handler mHandler = new Handler();
    private static boolean m_needRestartWifiAp;
    private static String[] ToastStrings = new String[] {"Photo captured!",
                                                         "Photo capture failed!",
                                                         "Video recording started!",
                                                         "Video recording stopped!"};

    public static Context m_context;

    private final static UsbIoManager.Listener m_Listener =
            new UsbIoManager.Listener()
            {
                @Override
                public void onRunError(Exception eA, int userDataA)
                {
                    Log.e(TAG, "onRunError Exception");
                    nativeDeviceException(userDataA, eA.getMessage());
                }

                @Override
                public void onNewData(final byte[] dataA, int userDataA)
                {
                    nativeDeviceNewData(userDataA, dataA);
                }
            };

    //  NATIVE C++ FUNCTION THAT WILL BE CALLED IF THE DEVICE IS UNPLUGGED
    private static native void nativeDeviceHasDisconnected(int userDataA);
    private static native void nativeDeviceException(int userDataA, String messageA);
    private static native void nativeDeviceNewData(int userDataA, byte[] dataA);

    // Native C++ functions called to log output
    public static native void qgcLogDebug(String message);
    public static native void qgcLogWarning(String message);

    private static native void nativeSendWifiApState(int state);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Constructor.  Only used once to create the initial instance for the static functions.
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public QGCActivity()
    {
        m_instance = this;
        m_openedDevices = new HashMap<Integer, UsbSerialDriver>();
        m_userData = new HashMap<Integer, Integer>();
        m_ioManager = new HashMap<Integer, UsbIoManager>();
        Log.i(TAG, "Instance created");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        PowerManager pm = (PowerManager)m_instance.getSystemService(Context.POWER_SERVICE);
        m_wl = pm.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK, "QGroundControl");
        //m_instance.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        m_wifiManager = (WifiManager)m_instance.getSystemService(Context.WIFI_SERVICE);
        m_wifiConfig = getWifiApConfiguration();
        setWifiApBand();
        m_Cm = (ConnectivityManager)m_instance.getSystemService(Context.CONNECTIVITY_SERVICE);
        m_needRestartWifiAp = false;
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    public void onInit(int status) {
    }

    @Override
    protected void onResume()
    {
        super.onResume();
    }

    @Override
    protected void onPause()
    {
        super.onPause();
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK) {
            Log.i(TAG, "BACK key down, but do nothing");
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }

    public static void acquireScreenWakeLock() {
        try{
            if(m_wl != null && !m_wl.isHeld()) {
                m_wl.acquire();
                Log.i(TAG, "SCREEN_BRIGHT_WAKE_LOCK acquired.");
            }
        } catch(Exception e) {
            Log.e(TAG, "Exception on acquire SCREEN_BRIGHT_WAKE_LOCK"+e);
        }
    }

    public static void releaseScreenWakeLock() {
        try {
            if(m_wl != null && m_wl.isHeld()) {
                m_wl.release();
                Log.i(TAG, "SCREEN_BRIGHT_WAKE_LOCK released.");
            }
        } catch(Exception e) {
           Log.e(TAG, "Exception on release SCREEN_BRIGHT_WAKE_LOCK"+e);
        }
    }

    public static void openDialPad() {
        Intent intent = new Intent(Intent.ACTION_DIAL);
        intent.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        m_instance.startActivity(intent);
    }

    public static void showToast(final int toastId) {
        m_instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (toastId < ToastStrings.length) {
                    Toast.makeText(m_instance, ToastStrings[toastId], Toast.LENGTH_SHORT).show();
                }
            }
        });
    }

    public static String getSdcardPath() {
        StorageManager storageManager = (StorageManager)m_instance.getSystemService(Activity.STORAGE_SERVICE);
        List<StorageVolume> volumes = storageManager.getStorageVolumes();
        Method mMethodGetPath;
        String path = "";
        for (StorageVolume vol : volumes) {
            try {
                mMethodGetPath = vol.getClass().getMethod("getPath");
            } catch (NoSuchMethodException e) {
                e.printStackTrace();
                continue;
            }
            try {
                path = (String) mMethodGetPath.invoke(vol);
            } catch (Exception e) {
                e.printStackTrace();
                continue;
            }

            if (vol.isRemovable() == true) {
                Log.i(TAG, "removable sd card mounted " + path);
                return path;
            } else {
                Log.i(TAG, "storage mounted " + path);
            }
        }
        return "";
    }

    public static void triggerMediaScannerScanFile(String path) {
        MediaScannerConnection.scanFile(m_instance, new String[] { path }, null, null);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Find all current devices that match the device filter described in the androidmanifest.xml and the
    //  device_filter.xml
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    private static boolean getCurrentDevices()
    {
        if (m_instance == null)
            return false;

        if (m_manager == null)
            m_manager = (UsbManager)m_instance.getSystemService(Context.USB_SERVICE);

        if (m_devices != null)
            m_devices.clear();

        m_devices = UsbSerialProber.findAllDevices(m_manager);

        return true;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  List all available devices that are not already open.  It returns the serial port info
    //  in a : separated string array.  Each string entry consists of the following:
    //
    //  DeviceName:Company:ProductId:VendorId
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static String[] availableDevicesInfo()
    {
        //  GET THE LIST OF CURRENT DEVICES
        if (!getCurrentDevices())
        {
            Log.e(TAG, "QGCActivity instance not present");
            return null;
        }

        //  MAKE SURE WE HAVE ENTRIES
        if (m_devices.size() <= 0)
        {
            //Log.e(TAG, "No USB devices found");
            return null;
        }

        if (m_openedDevices == null)
        {
            Log.e(TAG, "m_openedDevices is null");
            return null;
        }

        int countL = 0;
        int iL;

        //  CHECK FOR ALREADY OPENED DEVICES AND DON"T INCLUDE THEM IN THE COUNT
        for (iL=0; iL<m_devices.size(); iL++)
        {
            if (m_openedDevices.get(m_devices.get(iL).getDevice().getDeviceId()) != null)
            {
                countL++;
                break;
            }
        }

        if (m_devices.size() - countL <= 0)
        {
            //Log.e(TAG, "No open USB devices found");
            return null;
        }

        String[] listL = new String[m_devices.size() - countL];
        UsbSerialDriver driverL;
        String tempL;

        //  GET THE DATA ON THE INDIVIDUAL DEVICES SKIPPING THE ONES THAT ARE ALREADY OPEN
        countL = 0;
        for (iL=0; iL<m_devices.size(); iL++)
        {
            driverL = m_devices.get(iL);
            if (m_openedDevices.get(driverL.getDevice().getDeviceId()) == null)
            {
                UsbDevice deviceL = driverL.getDevice();
                tempL = deviceL.getDeviceName() + ":";

                if (driverL instanceof FtdiSerialDriver)
                    tempL = tempL + "FTDI:";
                else if (driverL instanceof CdcAcmSerialDriver)
                    tempL = tempL + "Cdc Acm:";
                else if (driverL instanceof Cp2102SerialDriver)
                    tempL = tempL + "Cp2102:";
                else if (driverL instanceof ProlificSerialDriver)
                    tempL = tempL + "Prolific:";
                else
                    tempL = tempL + "Unknown:";

                tempL = tempL + Integer.toString(deviceL.getProductId()) + ":";
                tempL = tempL + Integer.toString(deviceL.getVendorId()) + ":";
                listL[countL] = tempL;
                countL++;
                qgcLogDebug("Found " + tempL);
            }
        }

        return listL;
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Open a device based on the name.
    //
    //  Args:   nameA - name of the device to open
    //          userDataA - C++ pointer to the QSerialPort that is trying to open the device.  This is
    //                      used by the detector to inform the C++ side if it is unplugged
    //
    //  Returns:    ID number of opened port.  This number is used to reference the correct port in subsequent
    //              calls like close(), read(), and write().
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////
    public static int open(Context parentContext, String nameA, int userDataA)
    {
        int idL = BAD_PORT;

        m_context = parentContext;

        //qgcLogDebug("Getting device list");
        if (!getCurrentDevices())
            return BAD_PORT;

        //  CHECK THAT PORT IS NOT ALREADY OPENED
        if (m_openedDevices != null)
        {
            for (UsbSerialDriver driverL: m_openedDevices.values())
            {
                if (nameA.equals(driverL.getDevice().getDeviceName()))
                {
                    Log.e(TAG, "Device already opened");
                    return BAD_PORT;
                }
            }
        }
        else
            return BAD_PORT;

        if (m_devices == null)
            return BAD_PORT;

        //  OPEN THE DEVICE
        try
        {
            for (int iL=0; iL<m_devices.size(); iL++)
            {
                Log.i(TAG, "Checking device " + m_devices.get(iL).getDevice().getDeviceName() + " id: " + m_devices.get(iL).getDevice().getDeviceId());
                if (nameA.equals(m_devices.get(iL).getDevice().getDeviceName()))
                {
                    idL = m_devices.get(iL).getDevice().getDeviceId();
                    m_openedDevices.put(idL, m_devices.get(iL));
                    m_userData.put(idL, userDataA);

                    if (m_instance.m_UsbReceiver == null)
                    {
                        Log.i(TAG, "Creating new broadcast receiver");
                        m_instance.m_UsbReceiver= new BroadcastReceiver()
                        {
                            public void onReceive(Context contextA, Intent intentA)
                            {
                                String actionL = intentA.getAction();

                                if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(actionL))
                                {
                                    UsbDevice deviceL = (UsbDevice)intentA.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                                    if (deviceL != null)
                                    {
                                        if (m_userData.containsKey(deviceL.getDeviceId()))
                                        {
                                            nativeDeviceHasDisconnected(m_userData.get(deviceL.getDeviceId()));
                                        }
                                    }
                                }
                            }
                        };
                        //  TURN ON THE INTENT FILTER SO IT WILL DETECT AN UNPLUG SIGNAL
                        IntentFilter filterL = new IntentFilter();
                        filterL.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
                        m_instance.registerReceiver(m_instance.m_UsbReceiver, filterL);
                    }

                    m_openedDevices.get(idL).open();

                    //  START UP THE IO MANAGER TO READ/WRITE DATA TO THE DEVICE
                    UsbIoManager managerL = new UsbIoManager(m_openedDevices.get(idL), m_Listener, userDataA);
                    if (managerL == null)
                        Log.e(TAG, "UsbIoManager instance is null");
                    m_ioManager.put(idL, managerL);
                    m_Executor.submit(managerL);
                    Log.i(TAG, "Port open successful");
                    return idL;
                }
            }

            return BAD_PORT;
        }
        catch(IOException exA)
        {
            if (idL != BAD_PORT)
            {
                m_openedDevices.remove(idL);
                m_userData.remove(idL);

                if(m_ioManager.get(idL) != null)
                    m_ioManager.get(idL).stop();

                m_ioManager.remove(idL);
            }
            qgcLogWarning("Port open exception: " + exA.getMessage());
            return BAD_PORT;
        }
    }

    public static void startIoManager(int idA)
    {
        if (m_ioManager.get(idA) != null)
            return;

        UsbSerialDriver driverL = m_openedDevices.get(idA);

        if (driverL == null)
            return;

        UsbIoManager managerL = new UsbIoManager(driverL, m_Listener, m_userData.get(idA));
        m_ioManager.put(idA, managerL);
        m_Executor.submit(managerL);
    }

    public static void stopIoManager(int idA)
    {
        if(m_ioManager.get(idA) == null)
            return;

        m_ioManager.get(idA).stop();
        m_ioManager.remove(idA);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Sets the parameters on an open port.
    //
    //  Args:   idA - ID number from the open command
    //          baudRateA - Decimal value of the baud rate.  I.E. 9600, 57600, 115200, etc.
    //          dataBitsA - number of data bits.  Valid numbers are 5, 6, 7, 8
    //          stopBitsA - number of stop bits.  Valid numbers are 1, 2
    //          parityA - No Parity=0, Odd Parity=1, Even Parity=2
    //
    //  Returns:  T/F Success/Failure
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean setParameters(int idA, int baudRateA, int dataBitsA, int stopBitsA, int parityA)
    {
        UsbSerialDriver driverL = m_openedDevices.get(idA);

        if (driverL == null)
            return false;

        try
        {
            driverL.setParameters(baudRateA, dataBitsA, stopBitsA, parityA);
            return true;
        }
        catch(IOException eA)
        {
            return false;
        }
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Close the device.
    //
    //  Args:  idA - ID number from the open command
    //
    //  Returns:  T/F Success/Failure
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean close(int idA)
    {
        UsbSerialDriver driverL = m_openedDevices.get(idA);

        if (driverL == null)
            return false;

        try
        {
            stopIoManager(idA);
            m_userData.remove(idA);
            m_openedDevices.remove(idA);
            driverL.close();

            return true;
        }
        catch(IOException eA)
        {
            return false;
        }
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Write data to the device.
    //
    //  Args:   idA - ID number from the open command
    //          sourceA - byte array of data to write
    //          timeoutMsecA - amount of time in milliseconds to wait for the write to occur
    //
    //  Returns:  number of bytes written
    //
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    public static int write(int idA, byte[] sourceA, int timeoutMSecA)
    {
        UsbSerialDriver driverL = m_openedDevices.get(idA);

        if (driverL == null)
            return 0;

        try
        {
            return driverL.write(sourceA, timeoutMSecA);
        }
        catch(IOException eA)
        {
            return 0;
        }
        /*
        UsbIoManager managerL = m_ioManager.get(idA);

        if(managerL != null)
        {
            managerL.writeAsync(sourceA);
            return sourceA.length;
        }
        else
            return 0;
        */
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Determine if a device name if valid.  Note, it does not look for devices that are already open
    //
    //  Args:  nameA - name of device to look for
    //
    //  Returns: T/F
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean isDeviceNameValid(String nameA)
    {
        if (m_devices.size() <= 0)
            return false;

        for (int iL=0; iL<m_devices.size(); iL++)
        {
            if (m_devices.get(iL).getDevice().getDeviceName() == nameA)
                return true;
        }

        return false;
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Check if the device is open
    //
    //  Args:  nameA - name of device
    //
    //  Returns:  T/F
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean isDeviceNameOpen(String nameA)
    {
        if (m_openedDevices == null)
            return false;

        for (UsbSerialDriver driverL: m_openedDevices.values())
        {
            if (nameA.equals(driverL.getDevice().getDeviceName()))
                return true;
        }

        return false;
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Set the Data Terminal Ready flag on the device
    //
    //  Args:   idA - ID number from the open command
    //          onA - on=T, off=F
    //
    //  Returns:  T/F Success/Failure
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean setDataTerminalReady(int idA, boolean onA)
    {
        try
        {
            UsbSerialDriver driverL = m_openedDevices.get(idA);

            if (driverL == null)
                return false;

            driverL.setDTR(onA);
            return true;
        }
        catch(IOException eA)
        {
            return false;
        }
    }



    ////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Set the Request to Send flag
    //
    //  Args:   idA - ID number from the open command
    //          onA - on=T, off=F
    //
    //  Returns:  T/F Success/Failure
    //
    ////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean setRequestToSend(int idA, boolean onA)
    {
        try
        {
            UsbSerialDriver driverL = m_openedDevices.get(idA);

            if (driverL == null)
                return false;

            driverL.setRTS(onA);
            return true;
        }
        catch(IOException eA)
        {
            return false;
        }
    }



    ///////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Purge the hardware buffers based on the input and output flags
    //
    //  Args:   idA - ID number from the open command
    //          inputA - input buffer purge.  purge=T
    //          outputA - output buffer purge.  purge=T
    //
    //  Returns:  T/F Success/Failure
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    public static boolean purgeBuffers(int idA, boolean inputA, boolean outputA)
    {
        try
        {
            UsbSerialDriver driverL = m_openedDevices.get(idA);

            if (driverL == null)
                return false;

            return driverL.purgeHwBuffers(inputA, outputA);
        }
        catch(IOException eA)
        {
            return false;
        }
    }



    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get the native device handle (file descriptor)
    //
    //  Args:   idA - ID number from the open command
    //
    //  Returns:  device handle
    //
    ///////////////////////////////////////////////////////////////////////////////////////////
    public static int getDeviceHandle(int idA)
    {
        UsbSerialDriver driverL = m_openedDevices.get(idA);

        if (driverL == null)
            return -1;

        UsbDeviceConnection connectL = driverL.getDeviceConnection();
        if (connectL == null)
            return -1;
        else
            return connectL.getFileDescriptor();
    }



    //////////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Get the open usb serial driver for the given id
    //
    //  Args:  idA - ID number from the open command
    //
    //  Returns:  usb device driver
    //
    /////////////////////////////////////////////////////////////////////////////////////////////
    public static UsbSerialDriver getUsbSerialDriver(int idA)
    {
        return m_openedDevices.get(idA);
    }

    private void registerWifiBroadcast()
    {
        Log.d(TAG, "registerWifiBroadcast");
        IntentFilter filter = new IntentFilter();
        filter.addAction("android.net.wifi.WIFI_AP_STATE_CHANGED");
        registerReceiver(new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                String action = intent.getAction();
                if("android.net.wifi.WIFI_AP_STATE_CHANGED".equals(action)) {
                    int state = intent.getIntExtra(WifiManager.EXTRA_WIFI_STATE, 0);
                    nativeSendWifiApState(state);
                    if(state == 11 && m_needRestartWifiAp) {//WifiManager.WIFI_AP_STATE_DISABLED
                        m_needRestartWifiAp = false;
                        Log.d(TAG, "Restarting WifiAp due to prior config change.");
                        setWifiApEnabled(true);
                    }
                }
            }
        }, filter);
    }

    public static void registerBroadcast()
    {
        m_instance.registerWifiBroadcast();
    }

    public static void setNeedRestartWifiAp(boolean need)
    {
        m_needRestartWifiAp = need;
    }

    public static WifiConfiguration getWifiApConfiguration()
    {
        try {
            Method method = m_wifiManager.getClass().getMethod("getWifiApConfiguration");
            return (WifiConfiguration)method.invoke(m_wifiManager);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static boolean setWifiApConfiguration(String ssid, String password, int authType)
    {
        try {
            m_wifiConfig.SSID = ssid;
            m_wifiConfig.preSharedKey = password;
            m_wifiConfig.allowedKeyManagement.clear();
            if(authType == 1) {
                m_wifiConfig.allowedKeyManagement.set(4);//WifiConfiguration.KeyMgmt.WPA2_PSK
            } else if(authType == 0) {
                m_wifiConfig.allowedKeyManagement.set(WifiConfiguration.KeyMgmt.NONE);
            }
            Method method = m_wifiManager.getClass().getMethod("setWifiApConfiguration", WifiConfiguration.class);
            return (Boolean)method.invoke(m_wifiManager, m_wifiConfig);
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public static boolean setWifiApBand()
    {
        try {
            Field field = m_wifiConfig.getClass().getDeclaredField("AP_BAND_5GHZ");
            field.setAccessible(true);
            int AP_BAND_5GHZ = (int)field.get(m_wifiConfig);
            Field band_field = m_wifiConfig.getClass().getDeclaredField("apBand");
            band_field.setAccessible(true);
            int band = (int)band_field.get(m_wifiConfig);
            if(band != AP_BAND_5GHZ) {
                band_field.set(m_wifiConfig, AP_BAND_5GHZ);
                Method method = m_wifiManager.getClass().getMethod("setWifiApConfiguration", WifiConfiguration.class);
                method.invoke(m_wifiManager, m_wifiConfig);
            }
            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public static boolean setWifiApEnabled(boolean enabled)
    {
        try {
            Field field = m_Cm.getClass().getDeclaredField("TETHERING_WIFI");
            field.setAccessible(true);
            int TETHERING_WIFI = (int)field.get(m_Cm);
            Field mService = m_Cm.getClass().getDeclaredField("mService");
            mService.setAccessible(true);
            Object connService = mService.get(m_Cm);
            Class<?> connServiceClass = Class.forName(connService.getClass().getName());
            if(enabled) {
                Method startTethering = connServiceClass.getMethod("startTethering", int.class, ResultReceiver.class, boolean.class);
                startTethering.invoke(connService, TETHERING_WIFI, new ResultReceiver(mHandler) {
                    @Override
                    protected void onReceiveResult(int resultCode, Bundle resultData) {
                        super.onReceiveResult(resultCode, resultData);
                    }
                }, true);
            } else {
                Method stopTethering = connServiceClass.getMethod("stopTethering", int.class);
                stopTethering.invoke(connService, TETHERING_WIFI);
            }
            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public static void setCountryCode(String country, boolean persist)
    {
        Log.d(TAG, "setCountryCode: " + country + " " + persist);
        try {
            Method setCountryCode = m_wifiManager.getClass().getMethod("setCountryCode", String.class, boolean.class);
            setCountryCode.invoke(m_wifiManager, country, persist);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static String getCountryCode()
    {
        try {
            Method getCountryCode = m_wifiManager.getClass().getMethod("getCountryCode");
            return (String)getCountryCode.invoke(m_wifiManager);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}

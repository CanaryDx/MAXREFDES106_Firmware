#ifndef _MRD106_H_
#define _MRD106_H_

#include "mrd106_helper.h"
#include "mrd106_timers.h"
#include "mrd106_isr.h"

#include "sh_defs.h"
#include "sensorhub.h"

typedef struct mrd
{
    // Core hardware-control entry points for the MRD board
    int (*init)();
    int (*powerOn)();
    int (*powerOff)();
    int (*startMeasurement)();
    int (*stopMeasurement)();
    int (*enableSensors)(int en);
    int (*enableTimer)(int tmr, int en);
    int (*updateTimerPeriod)(int tmr, int period, int tmr_ps);

    // Public configuration/state API exposed to higher layers
    void (*setBatteryPercentage)(int battPer);
    int (*getBatteryPercentage)();
    void (*setChargingStatus)(bool en);
    bool (*getChargingStatus)();
    void (*setStatusLedEnable)(bool en);
    bool (*getStatusLedEnable)();
    void (*setMeasurementEnable)(bool en);
    bool (*getMeasurementEnable)();
    void (*setDebugLogEnable)(bool en);
    bool (*getDebugLogEnable)();
    void (*setFlashLogEnable)(bool en);
    bool (*getFlashLogEnable)();
    void (*setTempSensorEnable)(bool en);
    bool (*getTempSensorEnable)();
    void (*setTempSensorFreq)(int ind);
    int (*getTempSensorFreq)();
    void (*setBleSentEnable)();
    bool (*getBleSentEnable)();
    void (*setBleSentEvt)(int evt);
    int (*getBleSentEvt)();
    void (*setStopCmd)(int cmd);
    int (*getStopCmd)();


    // Registered sensor instances and accessor helper
    sensor_t *sensors[SH_NUM_SENSORS];
    sensor_t* (*getSensor)(int sensorNo);

    // Cached persistent state mirrored by the getter callbacks
    int p_isInit;
    int p_batteryPercentage;
    int p_chargingStatus;
    bool p_statusLedEnable;
    bool p_measurementEnable;
    bool p_debugLogEnable;
    bool p_flashLogEnable;
    bool p_tempSensorEnable;
    int p_tempSensorFreq;
    bool p_bleSentEnable;
    bool p_stopCmd;


    // Last BLE event sent to the host
    int e_bleSentEvt;
  
}mrd_t;

mrd_t* getMRD();


#endif

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_ADC/AP_ADC.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_InertialSensor ins;

static void display_offsets_and_scaling();
static void run_test();
static void run_calibrate();

void setup(void)
{
    hal.console->println("AP_InertialSensor startup...");

    ins.init(100);

    // display initial values
    display_offsets_and_scaling();
    hal.console->println("Complete. Reading:");

}

void loop(void)
{
    int16_t user_input;

    hal.console->println();
    hal.console->println(
            "Menu:\r\n"
            "    d) display offsets and scaling\r\n"
            "    l) level (capture offsets from level)\r\n"
            "    t) test\r\n"
            "    c) calibrate\r\n"
            "    r) reboot");

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while( hal.console->available() ) {
        user_input = hal.console->read();

        if( user_input == 'd' || user_input == 'D' ) {
            display_offsets_and_scaling();
        }

        if( user_input == 't' || user_input == 'T' ) {
            run_test();
        }

        if( user_input == 'c' || user_input == 'C' ) {
            run_calibrate();
        }

        if( user_input == 'r' || user_input == 'R' ) {
            hal.scheduler->reboot(false);
        }
    }
}

static void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf(
            "\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
            accel_offsets.x,
            accel_offsets.y,
            accel_offsets.z);
    hal.console->printf(
            "Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
            accel_scale.x,
            accel_scale.y,
            accel_scale.z);
    hal.console->printf(
            "Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
            gyro_offsets.x,
            gyro_offsets.y,
            gyro_offsets.z);
}

static void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    float length;
    uint8_t counter = 0;

    // flush any user input
    while( hal.console->available() ) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while( !hal.console->available() ) {

        // wait until we have a sample
        ins.wait_for_sample();

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        gyro = ins.get_gyro();

        length = accel.length();

        if (counter++ % 50 == 0) {
            // display results
            hal.console->printf("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \t Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f\n",
                    accel.x, accel.y, accel.z, length, gyro.x, gyro.y, gyro.z);
        }
    }

    // clear user input
    while( hal.console->available() ) {
        hal.console->read();
    }
}

static void run_calibrate()
{
    Vector3f accel;
    Vector3f gyro;

    int n_accels;

    n_accels = 1;

    hal.console->printf("Entered calibration mode\n");

    hal.console->printf("Making calibration file\n");
    // Probably want something to make sure all the offsets and stuff are at zero

    // Need probably six files perhaps
    for (int jj = 0; jj<6; jj++)
    {
        // Open the file
        char str[10];

        switch(jj) {
        case 0 :
            hal.console->printf("Z_down\n");
            sprintf(str, "%s", "Z_down.txt");
            break;

        case 1 :
            hal.console->printf("Z_up\n");
            sprintf(str, "%s", "Z_up.txt");
            break;

        case 2 :
            hal.console->printf("Y_down\n");
            sprintf(str, "%s", "Y_down.txt");
            break;

        case 3 :
            hal.console->printf("Y_up\n");
            sprintf(str, "%s", "Y_up.txt");
            break;

        case 4 :
            hal.console->printf("X_down\n");
            sprintf(str, "%s", "X_down.txt");
            break;

        case 5 :
            hal.console->printf("X_up\n");
            sprintf(str, "%s", "X_up.txt");
            break;

        default :
            hal.console->printf("Iteration %d!\n",jj);
            sprintf(str, "%d.txt", jj);
        }

        // Wait for user to confirm to take reading
        while( hal.console->available() ) {
            hal.console->read();
        }

        hal.console->printf("Press < return > to continue\n");
        while( !hal.console->available() ) {
            hal.scheduler->delay(20);
        }

        hal.console->printf("Starting recording\n");

        // Loop for each sensor
        for (int jj = 0; jj<n_accels; jj++)
        {

            // Start the data collection
            sprintf(str, "%d-%s",jj,str);
            FILE *f = fopen(str,"w");

            if (f == NULL)
            {
                printf("Error opening file!\n");
                exit(1);
            }

            fprintf(f,"Accelerometer Calibration File\n");
            fprintf(f,"=======================\n");

            // Write data points
            for (int ii = 0; ii<500; ii++)
            {
                // wait until we have a sample
                ins.wait_for_sample();

                // read samples from ins
                ins.update();


                accel = ins.get_accel(jj);  // const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
                gyro = ins.get_gyro(jj);

                //hal.console->printf("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f\n",accel.x, accel.y, accel.z);
                fprintf(f,"%f,%f,%f\n",accel.x, accel.y, accel.z);
            }

            fclose(f);
        }

        // Close the file


        hal.console->printf("Done file %d!\n\n",jj);
    }

    // Return
    return;
}

AP_HAL_MAIN();

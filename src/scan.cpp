
/*The MIT License (MIT)
 *
 * Copyright (c) 2017, Scanse, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ros/ros.h"
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#include <sweep/sweep.hpp>

int main(int argc, char *argv[]) try
{
    //Initialize Node and handles
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //Get Serial Parameters
    std::string serial_port;
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    int serial_baudrate;
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);

    //Get Scanner Parameters
    int rotation_speed;
    nh_private.param<int>("rotation_speed", rotation_speed, 5);

    int sample_rate;
    nh_private.param<int>("sample_rate", sample_rate, 500);

    //Get frame id Parameters
    std::string frame_id;
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

    //Setup Publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

    //Create Sweep Driver Object
    sweep::sweep device{serial_port.c_str()};

    //Send Rotation Speed
    device.set_motor_speed(rotation_speed);

    //Send Sample Rate
    device.set_sample_rate(sample_rate);

    ROS_INFO("expected rotation frequency: %d (Hz)", rotation_speed);

    //Start Scan
    device.start_scanning();

    // laser scan base msg
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = "laser_frame";
    msg.angle_min = -M_PI;
    msg.angle_max = M_PI;
    msg.angle_increment = 0.01;
    uint32_t ranges_size = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.range_min = 0;
    msg.range_max = 40;

    while (ros::ok())
    {
        //Grab Full Scan        
        const sweep::scan scan = device.get_scan();

        // write LaserScan
        msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
        msg.intensities.assign(ranges_size, 0);
        for (const sweep::sample& sample : scan.samples)
        {

            double angle = M_PI/180.*((float)sample.angle / 1000); //millidegrees to degrees to rad
            if(angle > M_PI)
                angle -= 2*M_PI;

            if(angle >= msg.angle_min && angle <= msg.angle_max)
            {
                int index = (angle - msg.angle_min) / msg.angle_increment;
                double range = sample.distance/100.;
                if(range < msg.ranges[index])
                {
                    msg.ranges[index] = range;
                    msg.intensities[index] = sample.signal_strength;
                }
            }
            else
                std::cout << "Angle not ok: " << 180/M_PI * angle << std::endl;
        }
        msg.header.stamp = ros::Time::now();

        scan_pub.publish(msg);

        ros::spinOnce();
    }

    //Stop Scanning & Destroy Driver
    device.stop_scanning();
}

    catch (const sweep::device_error& e) {
      std::cerr << "Error: " << e.what() << std::endl;
}

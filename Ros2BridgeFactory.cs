/**
 * Copyright (c) 2020-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using System;
using Simulator.Bridge.Data;
using Simulator.Bridge.Data.Lgsvl;
using Simulator.Bridge.Data.Ros;
// NOTE: DO NOT add using "Ros2.Ros" or "Ros2.Lgsvl" namespaces here to avoid
// NOTE: confusion between types. Keep them fully qualified in this file.

namespace Simulator.Bridge.Ros2
{
    [BridgeName("ROS2", "ROS2")]
    public class Ros2BridgeFactory : IBridgeFactory
    {
        public IBridgeInstance CreateInstance() => new Ros2BridgeInstance();

        public void Register(IBridgePlugin plugin)
        {
            // point cloud is special, as we use special writer for performance reasons
            plugin.AddType<PointCloudData>(Ros2Utils.GetMessageType<PointCloud2>());
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as Ros2BridgeInstance;
                    ros2Instance.AddPublisher<PointCloud2>(topic);
                    var writer = new Ros2PointCloudWriter(ros2Instance, topic);
                    return new Publisher<PointCloudData>((data, completed) => writer.Write(data, completed));
                }
            );

            RegPublisher<ImageData, CompressedImage>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<UncompressedImageData, Image>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<LaserScanData, LaserScan>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<CameraInfoData, CameraInfo>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<Detected3DObjectData, Detection3DArray>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<Detected2DObjectData, Detection2DArray>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<SignalDataArray, SignalArray>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<Data.CanBusData, Data.Lgsvl.CanBusData>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<UltrasonicData, Ultrasonic>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<GpsData, NavSatFix>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<GpsOdometryData, Odometry>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<ImuData, Imu>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<ClockData, Clock>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<VehicleOdometryData, VehicleOdometry>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<DetectedRadarObjectData, Data.Lgsvl.DetectedRadarObjectArray>(plugin, Ros2Conversions.ConvertFrom);

            RegSubscriber<Data.VehicleStateData, Data.Lgsvl.VehicleStateData>(plugin, Ros2Conversions.ConvertTo);
            RegSubscriber<Data.VehicleControlData, Data.Lgsvl.VehicleControlData>(plugin, Ros2Conversions.ConvertTo);
            RegSubscriber<Detected2DObjectArray, Detection2DArray>(plugin, Ros2Conversions.ConvertTo);
            RegSubscriber<Detected3DObjectArray, Detection3DArray>(plugin, Ros2Conversions.ConvertTo);
        }

        public void RegPublisher<DataType, BridgeType>(IBridgePlugin plugin, Func<DataType, BridgeType> converter)
        {
            plugin.AddType<DataType>(Ros2Utils.GetMessageType<BridgeType>());
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as Ros2BridgeInstance;
                    ros2Instance.AddPublisher<BridgeType>(topic);
                    var writer = new Ros2Writer<BridgeType>(ros2Instance, topic);
                    return new Publisher<DataType>((data, completed) => writer.Write(converter(data), completed));
                }
            );
        }

        public void RegSubscriber<DataType, BridgeType>(IBridgePlugin plugin, Func<BridgeType, DataType> converter)
        {
            plugin.AddType<DataType>(Ros2Utils.GetMessageType<BridgeType>());
            plugin.AddSubscriberCreator<DataType>(
                (instance, topic, callback) => (instance as Ros2BridgeInstance).AddSubscriber<BridgeType>(topic,
                    rawData => callback(converter(Ros2Serialization.Unserialize<BridgeType>(rawData)))
                )
            );
        }
    }
}

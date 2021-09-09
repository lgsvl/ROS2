/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;
using Simulator.Bridge.Data.Ros;
using Simulator.Bridge.Data.Lgsvl;
using Unity.Mathematics;
// NOTE: DO NOT add using "Ros2.Ros" or "Ros2.Lgsvl" namespaces here to avoid
// NOTE: confusion between types. Keep them fully qualified in this file.

namespace Simulator.Bridge.Ros2
{
    using Unity.Collections;
    using UnityEngine;
    using Pose = Data.Ros.Pose;
    using Quaternion = Data.Ros.Quaternion;
    using Time = Data.Ros.Time;
    using Vector3 = Data.Ros.Vector3;

    static class Ros2Conversions
    {
        public static CompressedImage ConvertFrom(ImageData data)
        {
            return new CompressedImage()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                format = "jpeg",
                data = new PartialByteArray()
                {
                    Array = data.Bytes,
                    Length = data.Length,
                },
            };
        }

        public static Image ConvertFrom(UncompressedImageData data)
        {
            var stride = data.Length / (data.Width * data.Height);
            var step = (uint) (stride * data.Width);
            byte isBigEndian;

            unsafe
            {
                var be = data.IsBigEndian;
                isBigEndian = *(byte*) (&be);
            }

            // Data from NativeArray has to be moved to managed array.
            // If BridgeMessageDispatcher was used to publish this message, this was done before and this path will
            // never execute (Bytes has valid data already, NativeArray is default).
            if (data.NativeArray != default)
            {
                if (data.Bytes == null || data.Bytes.Length != data.Length)
                {
                    // This is the slowest path - consider providing reusable Bytes array with valid size to avoid
                    // unnecessary allocations.
                    data.Bytes = new byte[data.Length];
                }

                NativeArray<byte>.Copy(data.NativeArray, 0, data.Bytes, 0, data.Length);
            }

            return new Image()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                width = (uint) data.Width,
                height = (uint) data.Height,
                step = step,
                encoding = data.Encoding,
                is_bigendian = isBigEndian,
                data = data.Bytes
            };
        }

        public static LaserScan ConvertFrom(LaserScanData data)
        {
            var count = data.Points.Length;

            if (data.RangesCache == null || data.RangesCache.Length != count)
                data.RangesCache = new float[count];

            if (data.IntensitiesCache == null || data.IntensitiesCache.Length != count)
                data.IntensitiesCache = new float[count];

            for (var i = 0; i < count; ++i)
            {
                var point = data.Points[i];

                var pos = new UnityEngine.Vector3(point.x, point.y, point.z);
                var intensity = point.w * 255f;

                pos = data.Transform.MultiplyPoint3x4(pos);
                var distance = pos.magnitude;
                if (distance < data.RangeMin || distance > data.RangeMax)
                {
                    distance = float.PositiveInfinity; // TODO: verify how LaserScan filters out points
                    intensity = 0;
                }

                var iOut = count - 1 - i;
                data.RangesCache[iOut] = distance;
                data.IntensitiesCache[iOut] = intensity;
            }

            var msg = new LaserScan()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                angle_min = data.MinAngle,
                angle_max = data.MaxAngle,
                angle_increment = data.AngleStep,
                time_increment = data.TimeIncrement,
                scan_time = data.ScanTime,
                range_min = data.RangeMin,
                range_max = data.RangeMax,
                ranges = data.RangesCache,
                intensities = data.IntensitiesCache
            };

            return msg;
        }

        public static CameraInfo ConvertFrom(CameraInfoData data)
        {
            return new CameraInfo()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                height = (uint)data.Height,
                width = (uint)data.Width,
                distortion_model = "plumb_bob",
                D = new double[5]
                {
                    (double)data.DistortionParameters[0],
                    (double)data.DistortionParameters[1],
                    0.0,
                    0.0,
                    (double)data.DistortionParameters[2],
                },
                K = new double[9]
                {
                    data.FocalLengthX, 0.0, data.PrincipalPointX,
                    0.0, data.FocalLengthY, data.PrincipalPointY,
                    0.0, 0.0, 1.0,
                },
                R = new double[9]
                {
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                },
                P = new double[12]
                {
                    data.FocalLengthX, 0.0, data.PrincipalPointX, 0.0,
                    0.0, data.FocalLengthY, data.PrincipalPointY, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                },
                binning_x = 0,
                binning_y = 0,
                roi = new RegionOfInterest()
                {
                    x_offset = 0,
                    y_offset = 0,
                    width = 0,
                    height = 0,
                    do_rectify = false,
                }
            };
        }

        public static Detection2DArray ConvertFrom(Detected2DObjectData data)
        {
            return new Detection2DArray()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                detections = data.Data.Select(d => new Detection2D()
                {
                    id = d.Id,
                    label = d.Label,
                    score = (float)d.Score,
                    bbox = new BoundingBox2D()
                    {
                        x = d.Position.x,
                        y = d.Position.y,
                        width = d.Scale.x,
                        height = d.Scale.y
                    },
                    velocity = new Twist()
                    {
                        linear = ConvertToVector(d.LinearVelocity),
                        angular = ConvertToVector(d.AngularVelocity),
                    }
                }).ToList(),
            };
        }

        public static Detected2DObjectArray ConvertTo(Detection2DArray data)
        {
            return new Detected2DObjectArray()
            {
                Data = data.detections.Select(obj =>
                    new Detected2DObject()
                    {
                        Id = obj.id,
                        Label = obj.label,
                        Score = obj.score,
                        Position = new UnityEngine.Vector2(obj.bbox.x, obj.bbox.y),
                        Scale = new UnityEngine.Vector2(obj.bbox.width, obj.bbox.height),
                        LinearVelocity = new UnityEngine.Vector3((float)obj.velocity.linear.x, 0, 0),
                        AngularVelocity = new UnityEngine.Vector3(0, 0, (float)obj.velocity.angular.z),
                    }).ToArray(),
            };
        }

        public static Detection3DArray ConvertFrom(Detected3DObjectData data)
        {
            var arr = new Detection3DArray()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                detections = new List<Detection3D>(),
            };

            foreach (var d in data.Data)
            {
                // Transform from (Right/Up/Forward) to (Forward/Left/Up)
                var position = d.Position;
                position.Set(position.z, -position.x, position.y);

                var orientation = d.Rotation;
                orientation.Set(-orientation.z, orientation.x, -orientation.y, orientation.w);

                var size = d.Scale;
                size.Set(size.z, size.x, size.y);

                d.AngularVelocity.z = -d.AngularVelocity.z;

                var det = new Detection3D()
                {
                    id = d.Id,
                    label = d.Label,
                    score = (float)d.Score,
                    bbox = new BoundingBox3D()
                    {
                        position = new Pose()
                        {
                            position = ConvertToPoint(position),
                            orientation = Convert(orientation),
                        },
                        size = ConvertToVector(size),
                    },
                    velocity = new Twist()
                    {
                        linear = ConvertToVector(d.LinearVelocity),
                        angular = ConvertToVector(d.AngularVelocity),
                    },
                };

                arr.detections.Add(det);
            }

            return arr;
        }

        public static SignalArray ConvertFrom(SignalDataArray data)
        {
            return new SignalArray()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                signals = data.Data.Select(d => new Signal()
                {
                    id = d.SeqId,
                    label = d.Label,
                    score = (float)d.Score,
                    bbox = new BoundingBox3D()
                    {
                        position = new Pose()
                        {
                            position = ConvertToPoint(d.Position),
                            orientation = Convert(d.Rotation),
                        },
                        size = ConvertToVector(d.Scale),
                    }
                }).ToList(),
            };
        }

        public static Data.Lgsvl.CanBusData ConvertFrom(Data.CanBusData data)
        {
            return new Data.Lgsvl.CanBusData()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                speed_mps = data.Speed,
                throttle_pct = data.Throttle,
                brake_pct = data.Braking,
                steer_pct = data.Steering,
                parking_brake_active = false,   // parking brake is not supported in Simulator side
                high_beams_active = data.HighBeamSignal,
                low_beams_active = data.LowBeamSignal,
                hazard_lights_active = data.HazardLights,
                fog_lights_active = data.FogLights,
                left_turn_signal_active = data.LeftTurnSignal,
                right_turn_signal_active = data.RightTurnSignal,
                wipers_active = data.Wipers,
                reverse_gear_active = data.InReverse,
                selected_gear = (data.InReverse ? Gear.GEAR_REVERSE : Gear.GEAR_DRIVE),
                engine_active = data.EngineOn,
                engine_rpm = data.EngineRPM,
                gps_latitude = data.Latitude,
                gps_longitude = data.Longitude,
                gps_altitude = data.Altitude,
                orientation = Convert(data.Orientation),
                linear_velocities = ConvertToVector(data.Velocity),
            };
        }

        public static Data.Lgsvl.DetectedRadarObjectArray ConvertFrom(DetectedRadarObjectData data)
        {
            var r = new Data.Lgsvl.DetectedRadarObjectArray()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                objects = new List<Data.Lgsvl.DetectedRadarObject>(),
            };

            foreach (var obj in data.Data)
            {
                r.objects.Add(new Data.Lgsvl.DetectedRadarObject()
                {
                    sensor_aim = ConvertToVector(obj.SensorAim),
                    sensor_right = ConvertToVector(obj.SensorRight),
                    sensor_position = ConvertToPoint(obj.SensorPosition),
                    sensor_velocity = ConvertToVector(obj.SensorVelocity),
                    sensor_angle = obj.SensorAngle,
                    object_position = ConvertToPoint(obj.Position),
                    object_velocity = ConvertToVector(obj.Velocity),
                    object_relative_position = ConvertToPoint(obj.RelativePosition),
                    object_relative_velocity = ConvertToVector(obj.RelativeVelocity),
                    object_collider_size = ConvertToVector(obj.ColliderSize),
                    object_state = (byte)obj.State,
                    new_detection = obj.NewDetection,
                });
            }

            return r;
        }

        public static Ultrasonic ConvertFrom(UltrasonicData data)
        {
            return new Ultrasonic()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                minimum_distance = data.MinimumDistance,
            };
        }

        public static NavSatFix ConvertFrom(GpsData data)
        {
            return new NavSatFix()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                status = new NavSatStatus()
                {
                    status = NavFixStatus.STATUS_FIX,
                    service = GpsServisType.SERVICE_GPS,
                },
                latitude = data.Latitude,
                longitude = data.Longitude,
                altitude = data.Altitude,

                position_covariance = new double[]
                {
                    0.0001, 0, 0,
                    0, 0.0001, 0,
                    0, 0, 0.0001,
                },

                position_covariance_type = CovarianceType.COVARIANCE_TYPE_DIAGONAL_KNOWN
            };
        }

        public static Odometry ConvertFrom(GpsOdometryData data)
        {
            return new Odometry()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },
                child_frame_id = data.ChildFrame,
                pose = new PoseWithCovariance()
                {
                    pose = new Pose()
                    {
                        position = new Point()
                        {
                            x = data.Easting,
                            y = data.Northing,
                            z = data.Altitude,
                        },
                        orientation = Convert(data.Orientation),
                    },
                    covariance = new double[]
                    {
                        0.0001, 0, 0, 0, 0, 0,
                        0, 0.0001, 0, 0, 0, 0,
                        0, 0, 0.0001, 0, 0, 0,
                        0, 0, 0, 0.0001, 0, 0,
                        0, 0, 0, 0, 0.0001, 0,
                        0, 0, 0, 0, 0, 0.0001
                    }
                },
                twist = new TwistWithCovariance()
                {
                    twist = new Twist()
                    {
                        linear = new Vector3()
                        {
                            x = data.ForwardSpeed,
                            y = 0.0,
                            z = 0.0,
                        },
                        angular = new Vector3()
                        {
                            x = 0.0,
                            y = 0.0,
                            z = -data.AngularVelocity.y,
                        }
                    },
                    covariance = new double[]
                    {
                        0.0001, 0, 0, 0, 0, 0,
                        0, 0.0001, 0, 0, 0, 0,
                        0, 0, 0.0001, 0, 0, 0,
                        0, 0, 0, 0.0001, 0, 0,
                        0, 0, 0, 0, 0.0001, 0,
                        0, 0, 0, 0, 0, 0.0001
                    }
                }
            };
        }

        public static VehicleOdometry ConvertFrom(VehicleOdometryData data)
        {
            return new VehicleOdometry()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                },
                velocity = data.Speed,
                front_wheel_angle = UnityEngine.Mathf.Deg2Rad * data.SteeringAngleFront,
                rear_wheel_angle = UnityEngine.Mathf.Deg2Rad * data.SteeringAngleBack,
            };
        }

        public static Detected3DObjectArray ConvertTo(Detection3DArray data)
        {
            return new Detected3DObjectArray()
            {
                Data = data.detections.Select(obj =>
                    new Detected3DObject()
                    {
                        Id = obj.id,
                        Label = obj.label,
                        Score = obj.score,
                        Position = Convert(obj.bbox.position.position),
                        Rotation = Convert(obj.bbox.position.orientation),
                        Scale = Convert(obj.bbox.size),
                        LinearVelocity = Convert(obj.velocity.linear),
                        AngularVelocity = Convert(obj.velocity.angular),
                    }).ToArray(),
            };
        }

        public static Data.VehicleControlData ConvertTo(Data.Lgsvl.VehicleControlData data)
        {
            float Deg2Rad = UnityEngine.Mathf.Deg2Rad;
            float MaxSteeringAngle = 39.4f * Deg2Rad;
            float wheelAngle = 0f;

            if (data.target_wheel_angle > MaxSteeringAngle)
            {
                wheelAngle = MaxSteeringAngle;
            }
            else if (data.target_wheel_angle < -MaxSteeringAngle)
            {
                wheelAngle = -MaxSteeringAngle;
            }
            else
            {
                wheelAngle = data.target_wheel_angle;
            }

            // ratio between -MaxSteeringAngle and MaxSteeringAngle
            var k = (float)(wheelAngle + MaxSteeringAngle) / (MaxSteeringAngle*2);

            // target_gear are not supported on simulator side

            return new Data.VehicleControlData()
            {
                TimeStampSec = Convert(data.header.stamp),
                Acceleration = data.acceleration_pct,
                Braking = data.braking_pct,
                SteerAngle = UnityEngine.Mathf.Lerp(-1f, 1f, k),
                SteerInput = data.target_wheel_angular_rate,
                TargetGear = (GearPosition)(int)data.target_gear,
            };
        }

        public static Data.VehicleStateData ConvertTo(Data.Lgsvl.VehicleStateData data)
        {
            return new Data.VehicleStateData()
            {
                Time = Convert(data.header.stamp),
                Blinker = (byte) data.blinker_state,
                HeadLight = (byte) data.headlight_state,
                Wiper = (byte) data.wiper_state,
                Gear = (byte) data.current_gear,
                Mode = (byte) data.vehicle_mode,
                HandBrake = data.hand_brake_active,
                Horn = data.horn_active,
                Autonomous = data.autonomous_mode_active,
            };
        }

        public static Imu ConvertFrom(ImuData data)
        {
            return new Imu()
            {
                header = new Header()
                {
                    stamp = Convert(data.Time),
                    frame_id = data.Frame,
                },

                orientation = Convert(data.Orientation),
                orientation_covariance = new double[9] { 0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001 },
                angular_velocity = ConvertToVector(data.AngularVelocity),
                angular_velocity_covariance = new double[9] { 0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001 },
                linear_acceleration = ConvertToVector(data.Acceleration),
                linear_acceleration_covariance = new double[9] { 0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001 },
            };
        }

        public static Clock ConvertFrom(ClockData data)
        {
            return new Clock()
            {
                clock = Convert(data.Clock),
            };
        }

        static Point ConvertToPoint(UnityEngine.Vector3 v)
        {
            return new Point() { x = v.x, y = v.y, z = v.z };
        }

        static Point ConvertToPoint(double3 d)
        {
            return new Point() { x = d.x, y = d.y, z = d.z };
        }

        static Vector3 ConvertToVector(UnityEngine.Vector3 v)
        {
            return new Vector3() { x = v.x, y = v.y, z = v.z };
        }

        static Quaternion Convert(UnityEngine.Quaternion q)
        {
            return new Quaternion() { x = q.x, y = q.y, z = q.z, w = q.w };
        }

        static UnityEngine.Vector3 Convert(Point p)
        {
            return new UnityEngine.Vector3((float)p.x, (float)p.y, (float)p.z);
        }

        static UnityEngine.Vector3 Convert(Vector3 v)
        {
            return new UnityEngine.Vector3((float)v.x, (float)v.y, (float)v.z);
        }

        static UnityEngine.Quaternion Convert(Quaternion q)
        {
            return new UnityEngine.Quaternion((float)q.x, (float)q.y, (float)q.z, (float)q.w);
        }

        public static Time Convert(double unixEpochSeconds)
        {
            long nanosec = (long)(unixEpochSeconds * 1e9);

            return new Time()
            {
                secs = (int)(nanosec / 1000000000),
                nsecs = (uint)(nanosec % 1000000000),
            };
        }

        public static double Convert(Time time)
        {
            return (double)time.secs + (double)time.nsecs * 1e-9;
        }
    }
}

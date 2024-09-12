import math
import time
import sys
import cv2
import pyrealsense2 as rs
import numpy as np
import logging

import imufusion


class RealsenseCamera:
    def __init__(
        self,
        width: int,
        height: int,
        depth_filter: bool = False,
        serial_number: str = "",
    ) -> None:
        """
        Initialize a RealSense pipeline with specified width and height for depth and color streams.

        Parameters:
        - width (int): Width of the streams.
        - height (int): Height of the streams.
        - serial_number (str): Serial number of the RealSense camera. Defaults to "" : it means that it will choose the camera automatically (useful when only one camera is connected).

        Returns:
        - pipeline: Initialized RealSense pipeline.
        """
        # Create a context object. This object owns the handles to all connected realsense devices
        self.pipeline = rs.pipeline()
        # Configure streams
        self.config = rs.config()
        if len(serial_number) > 0:
            self.config.enable_device(serial_number)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, 30)
        # for d435i
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)

        self.depth_filter = False
        if depth_filter:
            self.depth_filter = True
            self.dec_filter = rs.decimation_filter()
            self.spat_filter = rs.spatial_filter()
            self.temp_filter = rs.temporal_filter()
            self.hole_filter = rs.hole_filling_filter()

            # Configure filter parameters
            self.dec_filter.set_option(rs.option.filter_magnitude, 3)
            self.spat_filter.set_option(rs.option.filter_magnitude, 2)
            self.spat_filter.set_option(rs.option.filter_smooth_alpha, 1)
            self.spat_filter.set_option(rs.option.filter_smooth_delta, 50)
            self.temp_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
            self.temp_filter.set_option(rs.option.filter_smooth_delta, 20)
            self.hole_filter.set_option(rs.option.holes_fill, 1)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        # Using preset HighAccuracy for the depth sensor
        depth_sensor.set_option(rs.option.visual_preset, 3)

        # get the intrinsics of the depth sensor
        depth_profile = self.profile.get_stream(rs.stream.depth)
        depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
        fx, fy, cx, cy = (
            depth_intrinsics.fx,
            depth_intrinsics.fy,
            depth_intrinsics.ppx,
            depth_intrinsics.ppy,
        )
        self.calibration_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
        )

        self.first_frame = True
        self.alpha = 0.98
        self.totalgyroangleY = 0
        self.last_ts = 0
        self.ts = 0
        self.imu_fusion = imufusion.Ahrs()
        self.imu_fusion.reset()
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0

        time.sleep(2)  # Because the camera need time to be fully operationnal

    def get_points_and_colors(self):
        """
        Capture les coordonnées 3D et les couleurs associées à partir d'une caméra Intel RealSense.

        Args:
            pipeline (rs.pipeline): Objet de pipeline RealSense.
            filtered (bool): Indique si les données doivent être filtrées. Defaults to False.

        Returns:
            Tuple[np.ndarray, np.ndarray]: Tuple contenant les coordonnées 3D (vertices) et l'image couleur
        """
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        for frame in frames:

            pr = frame.get_profile()

            if (
                pr.stream_type() == rs.stream.accel
                and pr.format() == rs.format.motion_xyz32f
            ):
                accel = frame.as_motion_frame().get_motion_data()
                print(type(accel))
                print(pr.stream_type(), accel)

            if (
                pr.stream_type() == rs.stream.gyro
                and pr.format() == rs.format.motion_xyz32f
            ):
                gyro = frame.as_motion_frame().get_motion_data()
                print(type(gyro))
                print(pr.stream_type(), gyro)

        self.ts = frames.get_timestamp()

        if self.first_frame:
            self.first_frame = False
            self.last_ts = self.ts

            # accelerometer calculation
            accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
            accel_angle_x = math.degrees(
                math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z))
            )
            accel_angle_y = math.degrees(math.pi)
        else:
            dt = max((self.ts - self.last_ts)/1000, 0.001)
            self.last_ts = self.ts
            print(self.ts/1000)
            print(dt)

            vel_x = accel.x * dt
            vel_y = accel.y * dt
            vel_z = accel.z * dt
            
            self.pos_x += vel_x * dt
            self.pos_y += vel_y * dt
            self.pos_z += vel_z * dt

            self.imu_fusion.update_no_magnetometer(
                np.array([gyro.x, gyro.y, gyro.z]),
                np.array([accel.x, accel.y, accel.z]),
                1000.0 / dt,
            )  # 100 Hz sample rate
            print(dir(self.imu_fusion))
            print(np.round(self.imu_fusion.quaternion.to_euler()))
            print(f"pos_x: {np.round(self.pos_x,2)}, pos_y: {np.round(self.pos_y,2)}, pos_z: {np.round(self.pos_z,2)}")

        if self.depth_filter is True:
            # Post processing filters

            filtered_frame = depth_frame
            # Note the concatenation of output/input frame to build up a chain
            filtered_frame = self.dec_filter.process(filtered_frame)
            filtered_frame = self.spat_filter.process(filtered_frame)
            filtered_frame = self.temp_filter.process(filtered_frame)
            filtered_frame = self.hole_filter.process(filtered_frame)
            depth_frame = filtered_frame

        pc = rs.pointcloud()
        pc.map_to(depth_frame)
        points = pc.calculate(depth_frame)

        # Convert the coordinates to NumPy arrays
        vertices = np.array(points.get_vertices())
        color_image = np.array(color_frame.get_data())

        return (
            vertices.astype([("f0", "<f8"), ("f1", "<f8"), ("f2", "<f8")])
            .view(float)
            .reshape(vertices.shape + (-1,)),
            color_image,
        )

    def get_calibration_matrix(self) -> np.ndarray:
        """
        Recover the calibration matrix from a RealSense camera.

        Returns:
            np.ndarray: Calibration matrix.
        """
        return self.calibration_matrix

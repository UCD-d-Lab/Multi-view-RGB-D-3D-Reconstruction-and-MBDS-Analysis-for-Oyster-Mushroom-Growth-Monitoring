import pyrealsense2 as rs
import numpy as np
import cv2

class bag_frame:
    def __init__(self, bag_path, target_frame_No = 0):
        self.bag_path = bag_path
        self.target_frame_No = target_frame_No
        self.effective_frame_No = None
        self.serial_number = None
        self.frame = None
        self.type = None
        self.intrinsics = None
        self.depth_scale = None
        self.device_name = None        

    def judge_if_none(self):
        """
        Check if the frame is None.
        
        Returns:
            bool: True if frame is None, False otherwise.
        """
        return self.frame is None

    def extract_infrared(self, search_forward=200, return_none_on_fail=True):
        """
        Extract an infrared frame from the bag file.
        If the exact target_frame_No cannot be read, automatically fall back to the nearest readable frame.
        """
        if not self.judge_if_none():
            return self.frame is not None

        Pipeline = rs.pipeline()
        Config = rs.config()
        rs.config.enable_device_from_file(Config, self.bag_path, False)
        Config.enable_stream(rs.stream.infrared, 1)

        try:
            try:
                profile = Pipeline.start(Config)
            except RuntimeError as e:
                if return_none_on_fail:
                    self.frame = None
                    self.intrinsics = None
                    self.serial_number = None
                    return False
                else:
                    raise

            try:
                profile.get_device().as_playback().set_real_time(False)
            except Exception:
                pass

            last_valid = None
            last_valid_idx = None

            def safe_wait():
                try:
                    return Pipeline.wait_for_frames(1000)
                except RuntimeError:
                    return None

            # 1) jump to the target frame
            skip_n = max(0, self.target_frame_No - 1)
            for i in range(skip_n):
                fs = safe_wait()
                if fs is None:
                    continue
                ir = fs.get_infrared_frame()
                if ir:
                    last_valid = np.asanyarray(ir.get_data())
                    last_valid_idx = i + 1

            # 2) read the target frame
            fs = safe_wait()
            if fs:
                ir = fs.get_infrared_frame()
                if ir is not None:
                    self.frame = np.asanyarray(ir.get_data())
                    self.intrinsics = profile.get_stream(rs.stream.infrared).as_video_stream_profile().get_intrinsics()
                    device = profile.get_device()
                    self.serial_number = device.get_info(rs.camera_info.serial_number)
                    self.effective_frame_No = max(1, self.target_frame_No)
                    self.type = 'infrared'
                    return True

            # 3) search forward for several frames
            for j in range(1, search_forward + 1):
                fs = safe_wait()
                if fs is None:
                    continue
                ir = fs.get_infrared_frame()
                if ir is not None:
                    self.frame = np.asanyarray(ir.get_data())
                    self.intrinsics = profile.get_stream(rs.stream.infrared).as_video_stream_profile().get_intrinsics()
                    device = profile.get_device()
                    self.serial_number = device.get_info(rs.camera_info.serial_number)
                    self.effective_frame_No = max(1, self.target_frame_No) + j
                    self.type = 'infrared'
                    return True

            # 4) use the last valid frame
            if last_valid is not None:
                self.frame = last_valid
                self.intrinsics = profile.get_stream(rs.stream.infrared).as_video_stream_profile().get_intrinsics()
                device = profile.get_device()
                self.serial_number = device.get_info(rs.camera_info.serial_number)
                self.effective_frame_No = last_valid_idx
                self.type = 'infrared'
                return True

            # All failed
            if return_none_on_fail:
                self.frame = None
                self.intrinsics = None
                self.serial_number = None
                return False
            else:
                raise RuntimeError("Failed to read any infrared frame near target_frame_No.")
        finally:
            Pipeline.stop()


        
    def extract_depth(self, search_forward=10, max_tries=3, return_none_on_fail=True):
        if not self.judge_if_none():
            return True

        Pipeline = rs.pipeline()
        Config = rs.config()
        rs.config.enable_device_from_file(Config, self.bag_path, False)
        Config.enable_stream(rs.stream.depth)
        try:
            profile = Pipeline.start(Config)
        except RuntimeError as e:
            if return_none_on_fail:
                self.frame = None
                self.intrinsics = None
                self.serial_number = None
                return False
            else:
                raise        
        # profile = Pipeline.start(Config)

        try:
            try:
                profile.get_device().as_playback().set_real_time(False)
            except Exception:
                pass

            last_valid = None
            last_valid_idx = None

            def safe_wait():
                try:
                    return Pipeline.wait_for_frames(1000)
                except RuntimeError:
                    return None

            # 1) jump to target frame
            skip_n = max(0, self.target_frame_No - 1)
            tries = 0
            for i in range(skip_n):
                if tries >= max_tries: break
                fs = safe_wait()
                if fs is None:
                    tries += 1
                    continue
                df = fs.get_depth_frame()
                if df:
                    last_valid = np.asanyarray(df.get_data())
                    last_valid_idx = i + 1

            device = profile.get_device()
            self.device_name = device.get_info(rs.camera_info.name)
            depth_sensor = device.first_depth_sensor()
            self.depth_scale = float(depth_sensor.get_depth_scale())

            def set_depth_frame_and_meta(frame_np, eff_idx):
                self.frame = frame_np.astype(np.float32) * self.depth_scale
                self.intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
                self.serial_number = device.get_info(rs.camera_info.serial_number)
                self.effective_frame_No = eff_idx

            # 2) try target frame (limited time retry)
            tries = 0
            while tries < max_tries:
                fs = safe_wait()
                if fs is None:
                    tries += 1
                    continue
                df = fs.get_depth_frame()
                if df is not None:
                    depth_raw = np.asanyarray(df.get_data())
                    set_depth_frame_and_meta(depth_raw, max(1, self.target_frame_No))
                    self.type = 'depth'
                    return True
                tries += 1

            # 3) search forward for search_forward frames (each frame also limited time retry)
            found = False
            for j in range(1, search_forward + 1):
                tries = 0
                while tries < max_tries:
                    fs = safe_wait()
                    if fs is None:
                        tries += 1
                        continue
                    df = fs.get_depth_frame()
                    if df is not None:
                        depth_raw = np.asanyarray(df.get_data())
                        set_depth_frame_and_meta(depth_raw, max(1, self.target_frame_No) + j)
                        found = True
                        break
                    tries += 1
                if found:
                    break

            # 4) use the last valid frame before the target as a fallback
            if not found and last_valid is not None:
                set_depth_frame_and_meta(last_valid, last_valid_idx)
                self.type = 'depth'
                return True

            # all failed
            if return_none_on_fail:
                self.frame = None
                self.intrinsics = None
                self.serial_number = None
                return False
            else:
                raise RuntimeError("Failed to read any depth frame near target_frame_No.")
        finally:
            Pipeline.stop()



    def get_depth_to_color_extrinsics(self):

        device_extrinsics = {}

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device_from_file(self.bag_path, repeat_playback=False)
        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color)

        profile = pipeline.start(config)

        depth_stream = None
        color_stream = None
        for s in profile.get_streams():
            if s.stream_type() == rs.stream.depth:
                depth_stream = s
            elif s.stream_type() == rs.stream.color:
                color_stream = s

        if depth_stream is None or color_stream is None:
            pipeline.stop()
            raise RuntimeError("Depth or Color stream not found in profile")

        extrinsics = depth_stream.as_video_stream_profile().get_extrinsics_to(color_stream)

        device_extrinsics[self.serial_number] = extrinsics

        pipeline.stop()

        return device_extrinsics
   
    def get_color_aligned_to_depth(self, max_tries=3, return_none_on_fail=False):
        """
        Try to get an aligned color frame (to depth).
        - On intermittent 'Frame didn't arrive within 5000' errors, keep retrying up to max_tries.
        - If still failing and return_none_on_fail=True, return None instead of raising.
        """
        pipeline = rs.pipeline()
        config = rs.config()
        rs.config.enable_device_from_file(config, self.bag_path, False)
        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color)
        try:
            profile = pipeline.start(config)
        except RuntimeError as e:
            if return_none_on_fail:
                self.frame = None
                self.intrinsics = None
                self.serial_number = None
                return False
            else:
                raise
        # profile = pipeline.start(config)
        try:
            # playback to non-realtime to avoid frame drops (best-effort)
            try:
                profile.get_device().as_playback().set_real_time(False)
            except Exception:
                pass

            align = rs.align(rs.stream.depth)

            # skip to target vicinity; swallow timeouts while skipping
            skip_n = max(0, self.target_frame_No - 1)
            for _ in range(skip_n):
                try:
                    pipeline.wait_for_frames()
                except RuntimeError:
                    continue  # skip invalid frames during warm-up

            frames = None
            for _ in range(max_tries):
                try:
                    fs = pipeline.wait_for_frames()  # may raise RuntimeError
                except RuntimeError:
                    continue
                if fs and fs.get_depth_frame() and fs.get_color_frame():
                    frames = fs
                    break

            if frames is None:
                if return_none_on_fail:
                    return None
                raise RuntimeError("Failed to read a frameset with both depth and color from bag.")

            aligned = align.process(frames)
            depth_al = aligned.get_depth_frame()
            color_al = aligned.get_color_frame()
            if not depth_al or not color_al:
                if return_none_on_fail:
                    return None
                raise RuntimeError("Alignment produced empty frames.")

            color_np = np.asanyarray(color_al.get_data())
            H, W = depth_al.get_height(), depth_al.get_width()
            if color_np.shape[0] != H or color_np.shape[1] != W:
                color_np = cv2.resize(color_np, (W, H), interpolation=cv2.INTER_LINEAR)

            if self.serial_number is None:
                device = profile.get_device()
                self.serial_number = device.get_info(rs.camera_info.serial_number)

            return color_np  # (H, W, 3), uint8
        finally:
            pipeline.stop()


class Transformation:
    def __init__(self, rotation_matrix, translation_vector):
        self.pose_mat = np.zeros((4,4))
        self.pose_mat[:3,:3] = rotation_matrix
        self.pose_mat[:3,3] = translation_vector.flatten()
        self.pose_mat[3,3] = 1

    def apply_transformation(self, points):
        """
        Applies the transformation to the pointcloud

        Parameters:
        -----------
        points : array
            (3, N) matrix where N is the number of points

        Returns:
        ----------
        points_transformed : array
            (3, N) transformed matrix
        """
        assert(points.shape[0] == 3)
        n = points.shape[1]
        points_ = np.vstack((points, np.ones((1,n))))
        points_trans_ = np.matmul(self.pose_mat, points_)
        points_transformed = np.true_divide(points_trans_[:3,:], points_trans_[[-1], :])
        return points_transformed

    def inverse(self):
        """
        Computes the inverse transformation and returns a new Transformation object

        Returns:
        -----------
        inverse: Transformation

        """
        rotation_matrix = self.pose_mat[:3,:3]
        translation_vector = self.pose_mat[:3,3]

        rot = np.transpose(rotation_matrix)
        trans = - np.matmul(np.transpose(rotation_matrix), translation_vector)
        return Transformation(rot, trans)

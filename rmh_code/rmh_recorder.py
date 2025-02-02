# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time


class JointRecorder(object):
    def __init__(self, filename, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._start_time = time.time()
        self._done = False

        

    def _time_stamp(self):
        return time.time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        if self._filename:
            
            with open(self._filename, 'w') as f:
                f.write('time,')
                temp_str = '' if self._gripper else '\n'
                f.write(','.join([j for j in joints_right]) + ',' + temp_str)
                if self._gripper:
                    f.write(self.gripper_name+'\n')
                while not self.done():
                    # if self._gripper:
                    #     if self._cuff.upper_button():
                    #         self._gripper.open()
                    #     elif self._cuff.lower_button():
                    #         self._gripper.close()
                    # angles_right = [self._limb_right.joint_angle(j)
                    #                 for j in joints_right]
                    f.write("%f," % (self._time_stamp(),))
                    f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
                    if self._gripper:
                        f.write(str(self._gripper.get_position()) + '\n')
                    self._rate.sleep()

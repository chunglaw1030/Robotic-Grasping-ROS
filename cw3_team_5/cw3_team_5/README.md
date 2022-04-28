Author: Wing Chung Law (zceewcl@ucl.ac.uk), Eoin Burke (eoin.burke.21@ucl.ac.uk), Najib Soomro (najib.soomro.21@ucl.ac.uk)

Description: This package implements the code for COMP0129-S22 CW3.

Time spent: too long, equal contribution time

To run: 
1) Please enable octomap in moveit_panda_config/sensors_kinect_depthmap.yaml and sensors_kinect_pointcloud.yaml
2) Set <!-- <param name="octomap_resolution" type="double" value="0.025"/> --> in "sensor_manager.launch.xml"
2) catkin build + source devel/setup.bash
3) roslaunch cw1_team_5 run_solution.launch
4) open another terminal and run rosservice call /task 1 or task 2 or task 3


## License
LICENSE: MIT.  See LICENSE.txt

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2019-2022 Wing Chung Law except where specified

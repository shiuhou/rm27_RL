"""Build a separate pinned OpenVINS with observable native state publication.

Original research sources and installed binaries remain read-only. This local
instrumentation publishes initialization, feature-update counts, covariance,
velocity and native monotonic publication time. No truth is read or published.
It supplies missing evidence; its presence alone is not flight qualification.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from .dataset import digest
from .openvins import DEFAULT_INSTALL, ROOT


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('Pinned native instrumentation context changed')
    return text.replace(old,new)


def prepare(output,installation=DEFAULT_INSTALL):
    output=Path(output).resolve();install=json.loads(Path(installation).read_text())
    for path,expected in install['files_sha256'].items():
        if digest(path)!=expected:raise ValueError('Pinned research source changed: '+path)
    output.mkdir(parents=True,exist_ok=False)
    source=output/'source';workspace=output/'workspace';workspace.mkdir()
    shutil.copytree(install['source'],source,ignore=shutil.ignore_patterns('.git','build','install','log'))
    h=source/'ov_msckf/src/ros/ROS2Visualizer.h';c=source/'ov_msckf/src/ros/ROS2Visualizer.cpp'
    before={p.name:p.read_text() for p in [h,c]}
    hs=before[h.name].replace('#include <std_msgs/msg/float64.hpp>','#include <std_msgs/msg/float64.hpp>\n#include <std_msgs/msg/string.hpp>')
    hs=replace_once(hs,'  void publish_state();','  void publish_state();\n  void publish_rm27_state();')
    hs=replace_once(hs,'  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_poseimu;',
        '  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_poseimu;\n'
        '  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rm27_state;\n'
        '  uint64_t rm27_sequence = 0;\n  uint64_t rm27_epoch = 0;\n  double rm27_previous_state = -1;\n'
        '  int64_t rm27_camera_started_ns = 0;\n  int64_t rm27_camera_completed_ns = 0;\n'
        '  double rm27_camera_source_time = -1;')
    h.write_text(hs)
    cs=before[c.name].replace('#include "ROS2Visualizer.h"','#include "ROS2Visualizer.h"\n#include <chrono>\n#include <iomanip>\n#include <sstream>')
    cs=replace_once(cs,'  // Setup pose and path publisher','  pub_rm27_state = node->create_publisher<std_msgs::msg::String>("rm27_native_state", 100);\n\n  // Setup pose and path publisher')
    cs=replace_once(cs,'        _app->feed_measurement_camera(camera_queue.at(0));\n        visualize();',
        '        rm27_camera_source_time = camera_queue.at(0).timestamp;\n'
        '        rm27_camera_started_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();\n'
        '        _app->feed_measurement_camera(camera_queue.at(0));\n'
        '        rm27_camera_completed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();\n'
        '        publish_rm27_state();\n        visualize();')
    cs+='''
// RM27 local evidence publisher: no simulator state or truth input.
void ROS2Visualizer::publish_rm27_state() {
  auto state = _app->get_state();
  const bool initialized = _app->initialized();
  if (rm27_previous_state >= 0 && state->_timestamp < rm27_previous_state) ++rm27_epoch;
  rm27_previous_state = state->_timestamp;
  std::vector<std::shared_ptr<Type>> vars;
  vars.push_back(state->_imu->pose()->p());
  vars.push_back(state->_imu->pose()->q());
  vars.push_back(state->_imu->v());
  Eigen::Matrix<double, 9, 9> covariance = StateHelper::get_marginal_covariance(state, vars);
  std::ostringstream out; out << std::setprecision(17);
  out << "{\\"sequence\\":" << rm27_sequence++ << ",\\"epoch\\":" << rm27_epoch
      << ",\\"initialized\\":" << (initialized ? "true" : "false")
      << ",\\"state_time_s\\":" << state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0)
      << ",\\"camera_time_s\\":" << rm27_camera_source_time
      << ",\\"camera_processing_started_monotonic_ns\\":" << rm27_camera_started_ns
      << ",\\"camera_processing_completed_monotonic_ns\\":" << rm27_camera_completed_ns
      << ",\\"msckf_update_features\\":" << _app->get_good_features_MSCKF().size()
      << ",\\"slam_features\\":" << _app->get_features_SLAM().size()
      << ",\\"position\\":[";
  for (int i=0;i<3;i++) { if(i) out << ','; out << state->_imu->pos()(i); }
  out << "],\\"quaternion_xyzw\\":[";
  for (int i=0;i<4;i++) { if(i) out << ','; out << state->_imu->quat()(i); }
  out << "],\\"velocity_world\\":[";
  for (int i=0;i<3;i++) { if(i) out << ','; out << state->_imu->vel()(i); }
  out << "],\\"covariance_p_q_v\\":[";
  for (int i=0;i<9;i++) for (int j=0;j<9;j++) { if(i || j) out << ','; out << covariance(i,j); }
  const auto publication = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  out << "],\\"publish_monotonic_ns\\":" << publication << '}';
  std_msgs::msg::String msg; msg.data = out.str(); pub_rm27_state->publish(msg);
}
'''
    c.write_text(cs)
    import difflib
    patch=''.join(''.join(difflib.unified_diff(before[p.name].splitlines(keepends=True),p.read_text().splitlines(keepends=True),
        fromfile='a/ov_msckf/src/ros/'+p.name,tofile='b/ov_msckf/src/ros/'+p.name)) for p in [h,c])
    (output/'instrumentation.patch').write_text(patch)
    command=['docker','run','--rm','--network','none','--cpus','4','--memory','12g','--user',f'{os.getuid()}:{os.getgid()}',
        '-e','OPENBLAS_NUM_THREADS=1','-e','CMAKE_BUILD_PARALLEL_LEVEL=2',
        '-v',f'{source}:/upstream:ro','-v',f'{workspace}:/ws',install['image'],
        'bash','-c','source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --base-paths /upstream --packages-up-to ov_msckf --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON -DDISABLE_MATPLOTLIB=ON']
    (output/'launch.json').write_text(json.dumps(dict(command=command,parent_installation=install,
        instrumentation_sha256=digest(output/'instrumentation.patch'),runner_sha256=digest(__file__)),indent=2)+'\n')
    with (output/'build.log').open('w') as log:status=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    binary=workspace/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    result=dict(passed=status.returncode==0 and binary.exists(),returncode=status.returncode,
        run_type='native OpenVINS evidence-instrumentation build',flight_qualified=False)
    if result['passed']:
        pinned=dict(install,source=str(source),workspace=str(workspace),variant='pinned lifecycle patch plus RM27 native publication evidence',
            binary_sha256=digest(binary),instrumentation_sha256=digest(output/'instrumentation.patch'),
            files_sha256={str(p):digest(p) for p in [h,c,*list(source.glob('config/euroc_mav/*.yaml')),*list((workspace/'install').rglob('*.so'))]})
        (output/'installation.json').write_text(json.dumps(pinned,indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=prepare(a.output);return 0 if r['passed'] else 1

if __name__=='__main__':raise SystemExit(main())

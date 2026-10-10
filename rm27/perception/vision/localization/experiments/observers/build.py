"""Build observing example drivers externally against the qualified libraries.

No upstream source is edited. Only public API return/state reads and a CSV
writer are added to copies of the pinned examples. The ORB lifecycle library
patch is a separate pre-existing dependency, never applied by this builder.
"""
import argparse
import difflib
from pathlib import Path
import shutil
import subprocess

from ..backends import PINS
from ..common import require, write_json, file_hash


def replace_once(text, before, after):
    require(text.count(before) == 1, 'OBSERVER_SOURCE_MISMATCH', before[:80])
    return text.replace(before, after, 1)


def build(upstream, output):
    upstream, output = Path(upstream).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = output/'src'; source.mkdir()
    for directory, key in [('stella_vslam','stella_vslam'),('stella_vslam_examples','stella_vslam_examples'),('ORB_SLAM3','ORB-SLAM3')]:
        head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=upstream/'src'/directory,text=True).strip()
        require(head == PINS[key], 'OBSERVER_SOURCE_MISMATCH', directory)
    helper = Path(__file__).with_name('online_csv.hpp')
    shutil.copyfile(helper, source/helper.name)
    examples = upstream/'src/stella_vslam_examples'
    stella_file = examples/'src/run_tum_rgbd_slam.cc'
    original = stella_file.read_text()
    # Pin the example itself, including the observation point, to committed bytes.
    require(original == subprocess.check_output(['git','show','HEAD:src/run_tum_rgbd_slam.cc'],cwd=examples,text=True),
            'OBSERVER_SOURCE_MISMATCH','modified Stella example')
    stella = '#include "online_csv.hpp"\n#include "stella_vslam/publish/frame_publisher.h"\n'+original
    stella = stella.replace('    std::vector<double> track_times;', '    rm27_observer::Csv online_observer;\n    std::vector<double> track_times;', 1)
    stella = replace_once(stella, '            if (!rgb_img.empty() && (i % frame_skip == 0)) {',
                         '            std::shared_ptr<stella_vslam::Mat44_t> online_pose;\n            if (!rgb_img.empty() && (i % frame_skip == 0)) {')
    stella = replace_once(stella, '                slam->feed_monocular_frame(rgb_img, frame.timestamp_);',
                         '                online_pose = slam->feed_monocular_frame(rgb_img, frame.timestamp_);')
    needle = '            const auto tp_2 = std::chrono::steady_clock::now();'
    # First occurrence is the monocular loop; RGBD is deliberately uninstrumented.
    stella = stella.replace(needle, needle+'''
            const auto online_state = slam->get_frame_publisher()->get_tracking_state();
            online_observer.write(i, frame.timestamp_, tp_1, tp_2, online_state, online_pose.get());
''',1)
    (source/'stella_online.cc').write_text(stella)
    (output/'stella-example-observation.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), stella.splitlines(True),fromfile='run_tum_rgbd_slam.cc',tofile='stella_online.cc')))
    shutil.copytree(examples/'src/util',source/'util')
    orb_root=upstream/'src/ORB_SLAM3';orb_file=orb_root/'Examples/Monocular/mono_tum.cc'
    original=orb_file.read_text()
    require(original == subprocess.check_output(['git','show','HEAD:Examples/Monocular/mono_tum.cc'],cwd=orb_root,text=True),
            'OBSERVER_SOURCE_MISMATCH','modified ORB example')
    orb='#include "online_csv.hpp"\n'+original
    orb=replace_once(orb,'    // Main loop','    rm27_observer::Csv online_observer;\n    // Main loop')
    orb=replace_once(orb,'        SLAM.TrackMonocular(im,tframe);','        const auto online_pose_cw = SLAM.TrackMonocular(im,tframe);')
    needle='''        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();'''
    orb=replace_once(orb,needle,'''        const int online_state = SLAM.GetTrackingState();
        const Eigen::Matrix4d online_matrix = online_pose_cw.matrix().cast<double>();
        online_observer.write(ni, tframe, t1, t2, std::to_string(online_state), &online_matrix);

'''+needle)
    (source/'orb_online.cc').write_text(orb)
    (output/'orb-example-observation.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),orb.splitlines(True),fromfile='mono_tum.cc',tofile='orb_online.cc')))
    cmake='''cmake_minimum_required(VERSION 3.16)
project(rm27_online_observers LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(stella_vslam REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(stella_online src/stella_online.cc src/util/tum_rgbd_util.cc)
target_include_directories(stella_online PRIVATE "@EXAMPLES@/3rd/popl/include" "@EXAMPLES@/3rd/filesystem/include")
target_link_libraries(stella_online PRIVATE stella_vslam::stella_vslam opencv_imgcodecs opencv_videoio)
add_executable(orb_online src/orb_online.cc)
target_compile_definitions(orb_online PRIVATE COMPILEDWITHC11)
target_include_directories(orb_online PRIVATE "@ORB@" "@ORB@/include" "@ORB@/include/CameraModels" "@ORB@/Thirdparty/Sophus" ${Pangolin_INCLUDE_DIRS})
target_link_libraries(orb_online PRIVATE "@ORB@/lib/libORB_SLAM3.so" ${Pangolin_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)
'''.replace('@EXAMPLES@',str(examples)).replace('@ORB@',str(orb_root))
    (output/'CMakeLists.txt').write_text(cmake)
    commands=[['cmake','-S',str(output),'-B',str(output/'build'),'-GNinja','-DCMAKE_BUILD_TYPE=Release',
               '-DCMAKE_PREFIX_PATH='+str(upstream/'install')+';'+str(upstream/'sysroot/usr')],
              ['cmake','--build',str(output/'build'),'-j','2']]
    for i, command in enumerate(commands):
        with (output/f'build-{i}.log').open('w') as log:
            code=subprocess.run(command,stdout=log,stderr=log).returncode
        require(code==0,'OBSERVER_BUILD_FAILED',str(output/f'build-{i}.log'))
    write_json(output/'build.json',dict(commands=commands,upstream=str(upstream),
               sources_sha256={str(p.relative_to(output)):file_hash(p) for p in source.rglob('*') if p.is_file()},
               observer_helper_sha256=file_hash(helper),builder_sha256=file_hash(Path(__file__)),
               observation_only=True,estimator_libraries_rebuilt=False))
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(); print(build(args.upstream,args.output))

if __name__=='__main__': main()

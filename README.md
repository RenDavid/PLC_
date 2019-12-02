The project is based on the COLMAP framework. Please refer to https://colmap.github.io/ for more information like installation and tutorial.
The usage of this application is similair to that of COLMAP. Here is one example of command-lines applying PLC on Linux. The main differerences from COLMAP command-lines lay on the 'colmap patch_match_stereo' block where we add options for applying pyramid architecture and local consistency.

----------------------------------------------------------------------------------------------------------------
LD_LIBRARY_PATH=/opt/Qt5.12.0/5.12.0/gcc_64/lib/
iname=$1/
gpu=$2
filter=$3
depth_error=$4
reproj_error=$5
PROJECT_PATH=${iname}result
DATABASE=${iname}sample_reconstruction.db

mkdir ${PROJECT_PATH}
mkdir ${PROJECT_PATH}/images
cp -n ${iname}*.jpg ${PROJECT_PATH}/images

colmap feature_extractor \
    --database_path ${DATABASE} \
    --image_path ${PROJECT_PATH}/images \
    --ImageReader.camera_model RADIAL \
    --ImageReader.single_camera 1 \
	--SiftExtraction.use_gpu 1 \
	--SiftExtraction.gpu_index ${gpu}
	
colmap exhaustive_matcher \
    --database_path ${DATABASE} \
    --SiftMatching.use_gpu 1 \
    --SiftMatching.max_num_matches 10000 \
    --SiftMatching.gpu_index ${gpu}
    
mkdir ${PROJECT_PATH}/sparse
colmap mapper \
    --database_path ${DATABASE} \
    --image_path ${PROJECT_PATH}/images \
    --output_path ${PROJECT_PATH}/sparse

mkdir ${PROJECT_PATH}/dense

colmap image_undistorter \
    --image_path ${PROJECT_PATH}/images \
    --input_path ${PROJECT_PATH}/sparse/0/ \
    --output_path ${PROJECT_PATH}/dense \
    --output_type COLMAP

colmap patch_match_stereo \
    --workspace_path ${PROJECT_PATH}/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.filter true \
    --PatchMatchStereo.geom_consistency true \
    --PatchMatchStereo.gpu_index ${gpu} \
    --PatchMatchStereo.pyramid_stereo_match 3 \
    --PatchMatchStereo.pyramid_stereo_match_l 0 \
    --PatchMatchStereo.smoothness 0.99 \
    --PatchMatchStereo.num_iterations 5

colmap stereo_fusion \
    --workspace_path ${PROJECT_PATH}/dense \
    --workspace_format COLMAP \
    --input_type geometric \
    --StereoFusion.min_num_pixels 5 \
    --StereoFusion.max_num_pixels 1000 \
    --StereoFusion.max_depth_error $depth_error \
    --StereoFusion.max_reproj_error $reproj_error \
    --output_path ${PROJECT_PATH}/dense/fused_1.ply
    
---------------------------------------------------------------------------------------------------------------------

If you find this work helpful for you, please cite our paper and "Pixelwise View Selection for Unstructured Multi-View Stereo".

@article {10.1111:cgf.13841,
journal = {Computer Graphics Forum},
title = {{Pyramid Multi-View Stereo with Local Consistency}},
author = {Liao, Jie and Fu, Yanping and Yan, Qingan and Xiao, Chunxia},
year = {2019},
publisher = {The Eurographics Association and John Wiley & Sons Ltd.},
ISSN = {1467-8659},
DOI = {10.1111/cgf.13841}
}

@inproceedings{schoenberger2016mvs,
    author={Sch\"{o}nberger, Johannes Lutz and Zheng, Enliang and Pollefeys, Marc and Frahm, Jan-Michael},
    title={Pixelwise View Selection for Unstructured Multi-View Stereo},
    booktitle={European Conference on Computer Vision (ECCV)},
    year={2016},
}

cd ..
git clone --depth 1 https://github.com/cnr-isti-vclab/vcglib.git

conda install cmake -y

conda install -c conda-forge eigen opencv ceres-solver \
  cgal boost glfw nanoflann libjxl glad -y

cd open-mvs

mkdir make
cd make

cmake .. \
  -DVCG_ROOT=${PWD}/../../vcglib \
  -DOpenMVS_USE_CUDA=ON \
  -DCMAKE_PREFIX_PATH=${CONDA_PREFIX}

make -j

make install

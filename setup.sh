cd ..
git clone --depth 1 https://github.com/cnr-isti-vclab/vcglib.git

conda install cmake -y
conda install -y -c conda-forge gcc=11 gxx=11
conda install -c conda-forge eigen opencv ceres-solver \
  cgal boost glfw nanoflann libjxl glad openmp -y

cd open-mvs

rm -rf make
mkdir make
cd make

cmake .. \
  -DVCG_ROOT=${PWD}/../../vcglib \
  -DOpenMVS_USE_CUDA=ON \
  -DCMAKE_PREFIX_PATH=${CONDA_PREFIX} \
  -DCMAKE_LIBRARY_PATH=${CONDA_PREFIX}/lib

make -j

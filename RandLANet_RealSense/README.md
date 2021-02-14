# RandLaNet_RealSense

### (1) Setup
 
- Clone the repository 
```
git clone
```
- Conda Installation
https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html
Downlaod 64-Bit (x86) Installer (529 MB)
Cryptographic hash verification
```
sha256sum filename
```
Ausführen
```
bash Anaconda-latest-Linux-x86_64.sh
```
- Setup python environment
```
conda create -n randlanet python=3.5
source activate randlanet
pip install -r helper_requirements.txt
sh compile_op.sh
```

### (2) RealSense
RealSense dataset can be found 
<a href="https://drive.google.com/drive/folders/1Nr5vaNY-JVY5tXSAY0KzCT8Tdia7q6I0?usp=sharing">here</a>. 
Uncompress the folder and move it to 
`/data/RealSense`.

- Preparing the dataset:
```
python utils/data_prepare_RealSense.py
```
- Start training:
```
python main_RealSense.py --gpu 0 --mode train
```
- Start testing:
```
python main_RealSense.py --gpu 0 --mode test
```
- Move all the generated results (*.ply) in `/test` folder to `/data/RealSense/results`, calculate the final mean IoU results:
```
python utils/6_fold_cv.py
```



### Acknowledgment
-  The code refers to <a href="https://github.com/QingyongHu/RandLA-Net">RandLaNet</a>. The Network was customized to fit the Intel RealSense data.
-  Part of their code refers to <a href="https://github.com/jlblancoc/nanoflann">nanoflann</a> library and the the recent work <a href="https://github.com/HuguesTHOMAS/KPConv">KPConv</a>.

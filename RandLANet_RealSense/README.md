# RandLaNet_RealSense

### Setup
 
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
- Conda Python Umgebung
```
conda create -n randlanet python=3.5
source activate randlanet
pip install -r helper_requirements.txt
sh compile_op.sh
```
### Datenerstellung
Mit Hilfe des <a href="https://github.com/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor">Semantic Segmentation Editor</a>
```
cd semantic-segmentation-editor-x.x.x
meteor npm install
meteor npm start
```
### RealSense
<a href="https://drive.google.com/drive/folders/1Nr5vaNY-JVY5tXSAY0KzCT8Tdia7q6I0?usp=sharing">Datensatz</a>
Entpacken und in `/data/RealSense` legen.

<a href="https://drive.google.com/drive/folders/1Nr5vaNY-JVY5tXSAY0KzCT8Tdia7q6I0?usp=sharing">Vortrainierte Modelle</a>
Entpacken und in `/data/RealSense` legen.

- Vorbereitung des Datensatzes:
```
python utils/data_prepare_RealSense.py
```
- Start Training:
```
python main_RealSense.py --gpu 0 --mode train
```
- Start Test:
```
python main_RealSense.py --gpu 0 --mode test
```
- Alle generierten Punktwolken (*.ply) in `/test` Ordner zu `/data/RealSense/results`, berechnen des finalen mean IoU Ergebnis:
```
python utils/6_fold_cv.py
```

### Error Handling
Bei:
```
Traceback (most recent call last):
  File "utils/data_prepare_RealSense.py", line 3, in <module>
    from pypcd import pypcd 
  File "/home/joshua/anaconda3/envs/randlanet/lib/python3.5/site-packages/pypcd/pypcd.py", line 15, in <module>
    import cStringIO as sio
ImportError: No module named 'cStringIO'
```
An entsprechender Stelle dies einfügen:
```
try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO
```
### Acknowledgment
-  The code refers to <a href="https://github.com/QingyongHu/RandLA-Net">RandLaNet</a>. The Network was customized to fit the Intel RealSense data.
-  Part of their code refers to <a href="https://github.com/jlblancoc/nanoflann">nanoflann</a> library and the the recent work <a href="https://github.com/HuguesTHOMAS/KPConv">KPConv</a>.

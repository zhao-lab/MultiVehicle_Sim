# DPGP for Multi-Vehicle Interaction Scenario Extraction
This repo provides the python implementation of DPGP algorithm using Gaussian Process to represent multi-vehicle driving scenarios with Dirichlet Process adapting cluster numbers. <br>
The python version code is implemented by Vinay Varma Kalidindi, Mengdi Xu @SafeAI lab in CMU. <br>
Initial MATLAB code implemented by Yaohui Guo, Vinay Varma Kalidindi. <br>

### Paper Reference:
Modeling Multi-Vehicle Interaction Scenarios Using Gaussian Random Field <br>
https://arxiv.org/pdf/1906.10307.pdf


### Improvement:
The code structure is more clear and can easily be implemented for various applications. <br>
Thanks members of SafeAI lab for discussion! <br>


#### Input:

frames: list with element as object defined in frame.py <br>

#### Output:

Mixture model as defined in mixtureModel.py <br>

#### Implement:
Train DPGP: python main.py <br>
Visualization: python pattern_vis.py

#### Required python packages:
numpy         == 1.16.4 <br>
scipy         == 1.3.1 <br>
scikit-learn  == 0.21.2 <br>
pandas        == 0.25.0 <br>
Some others: <br>
math  <br>
multiprocessing <br>
functools <br>
pickle <br>

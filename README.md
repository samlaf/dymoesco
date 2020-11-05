# dymoesco - Dynamics Modeling, Estimation and Control

dymoesco is my attempt at building a general purpose robotics library to speed up the testing of simple new ideas. It is also great as a teaching platform. [Drake](https://drake.mit.edu/) should be used for any serious project. But for prototyping or testing quick ideas, its API can feel inflexible and overly complicated. dymoesco also has matplotlib plotting and animations, which when a full-fledged rendering engine is not needed, makes for nice and easy gif generation. In a nutshell, dymoesco is to Drake what [turtle](https://docs.python.org/3/library/turtle.html) is to [unity](https://unity.com/).

### Installation

dymoesco is structured as a [namespace package](https://packaging.python.org/guides/packaging-namespace-packages/), which `estimation`, `control`, and `dynamics` as the main subpackages.
Installing dymoesco works just like any other package

```shell
git clone https://github.com/samlaf/dymoesco.git
pip install -r requirements.txt .
```

### Docs
Sphinx documentation is found in the /docs directory.
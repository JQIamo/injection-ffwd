# Injection FFWD

Based on this paper: [Active stabilization of a diode laser injection lock](http://dx.doi.org/10.1063/1.4953589)
arXiv version: https://arxiv.org/abs/1602.03504

Basic idea: measure the laser peak height on a fabry perot cavity and feed forward to the injection current to maximize the peak height (ie, maintain it in a stable injection locked region)

Implements a crude servo algorithm with auto-relock feature.

Todo:

* status of lock to computer control (pause cycle if in auto-relock)
* ...?

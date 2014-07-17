====================================
Gait Pattern Generator for Humanoids
====================================

:Author: Manuel Kudruss
:Address: Im Neuenheimer Feld 368
          69120 Heidelberg
          Germany
:Contact: manuel.kudruss@iwr.uni-heidelberg.de
:Authors: Manuel Kudruss; Maximilien Naveau ...
:organization: Interdisciplinary Center for Scientific Computing
:date: $Date: 2014-08-17 15:12:43 +0000 (Thu, 17 Aug 2014)$
:status: This is a "work in progress"
:Dedication:

    Roboticists that want their humanoid to walk

:abstract:

General
=======

This is a collection of different approaches to gait pattern generation for humanoid robots.
A paper is available here ... giving an overview of some of the algorithms in the module.
The algorithms itself are documented with docstrings. Usage examples are given in the test cases.

Third Party Software
====================

The algorithms in this module use different third party libraries to solve the resulting
optimization problems for the different pattern generator types.

qpOASES
-------

Download the QP solver qpOASES from here (`here <http://set.kuleuven.be/optec/Software/qpOASES-OPTEC>`__). Academic license is free.

Follow the instructions to install the pythong interface and add qpOASES to your PYTHONPATH in your bash.rc.


gurobi
------

For the QCQP version of the pattern generator we use gurobi as solver (`here <http://http://www.gurobi.com/en>`__). Academic license is free.

Because their python interface is crappy. Please Download a real one from ...

Follow the instructions on how to install the python interface for gurobi.


SNOPT7
------

For the nonlinear version of the pattern generator the SQP implementation SNOPT7 is used. Get it (`here <http://www.sbsi-sol-optimize.com/asp/sol_products_snopt_desc.htm>`__).
A proper license is needed to use this software.

The python interface is available through ....

Follow the instructions on how to install the python interface for SNOPT7.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ethercat_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.0 (2021-03-03)
------------------

1.8.19 (2019-07-26)
-------------------
* Make sure to include the correct boost libraries.
  This follows the principle of "include what you use", and
  also should in theory fix the problems on the build farm.
  (`#76 <https://github.com/PR2/pr2_ethercat_drivers/issues/76>`_)
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

1.8.18 (2019-03-12)
-------------------
* fix order of Changelog contents
* Merge pull request `#75 <https://github.com/pr2/pr2_ethercat_drivers/issues/75>`_ from k-okada/add_travis
  update travis.yml and fix for kinetic/melodic
* fix gcc 8 error narrowing char
  explicitly mark the char array as unsigned.
* add build/run depend to tinyxml
* fixed compile error
* Merge pull request `#73 <https://github.com/pr2/pr2_ethercat_drivers/issues/73>`_ from TAMS-Group/pr-kinetic-add-ft-frame
  add frame_id to f/t wrench messages
* add frame_id to f/t wrench messages
  This adds the original frame name to the messages
  from the force torque sensors.
  Fixes `#66 <https://github.com/pr2/pr2_ethercat_drivers/issues/66>`_
* Contributors: David Feil-Seifer, Kei Okada, v4hn

1.8.17 (2018-02-14)
-------------------
* Merge pull request `#69 <https://github.com/PR2/pr2_ethercat_drivers/issues/69>`_ from goretkin/patch-1
  removed lingering hard-coded constant
* Merge pull request `#70 <https://github.com/PR2/pr2_ethercat_drivers/issues/70>`_ from k-okada/orp
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* removed lingering constant
  MAX_FT_SAMPLES is currently set to 4, so this doesn't affect the behavior.
* Contributors: Gustavo Goretkin, Kei Okada

1.8.15 (2015-06-19)
-------------------
* Added install rules
* Contributors: TheDash

1.8.12 (2015-02-11)
-------------------

1.8.11 (2014-09-30)
-------------------

1.8.9 (2014-09-29)
------------------

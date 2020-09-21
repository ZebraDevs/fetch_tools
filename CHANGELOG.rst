^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2020-09-20)
------------------
* Update dependencies from py2 to py3
* Contributors: Eric Relson

0.3.0 (2020-09-20)
------------------
* Convert to python3 + ROS noetic
* Text improvements to debug_snapshot tool
* Several 18.04 fixes to debug_snapshot tool
* Contributors: Alex Moriarty, Eric Relson, Nick Walker, Russell Toris

0.2.1 (2019-03-26)
------------------
* Fixups for indigo->melodic; package format (`#13 <https://github.com/fetchrobotics/fetch_tools/issues/13>`_)
  - Updates to debug_snapshot
  - Warning about common failure case when using debug_snapshot
* Contributors: Eric Relson

0.2.0 (2018-07-11)
------------------
* updates ownership
* Update create_account.py
* Expand hardware info retrieval and add read_board
* Contributors: Alex Henning, Michael Ferguson, Michael Hwang, Russell Toris

0.1.4 (2016-04-12)
------------------
* Added FETCH_USER to sf/sfr & additional usage notes
* Updated readme to detail robothostname.local usage with fetch_tools and how to set ports with ul
* Contributors: Alex Henning, Eric Relson

0.1.3 (2016-03-24)
------------------
* Added upstart conf files to debug-snapshot zip
* Contributors: Aaron Blasdel

0.1.2 (2016-02-29)
------------------
* Added three commands:
  - `fetch debug-snapshot`
  - `fetch pull`
  - `fetch workspace-status`
* Changed one command:
  - `fetch sync` is now `fetch push`
* Improved one command:
  - `fetch run` now supports multiple workspaces
* Contributors: Alex Henning, Michael Ferguson, Aaron Blasdel

0.1.1 (2015-07-31)
------------------
* Initial implementation of fetch_tools
  Includes four commands:
  - `fetch create-account`
  - `fetch sync`
  - `fetch run`
  - `fetch lint`
  Includes five aliases:
  - uf
  - ufr
  - ul
  - sf
  - sfr
* Initial commit
* Contributors: Alex Henning, Michael Ferguson

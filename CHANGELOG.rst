^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fetch_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2018-07-11)
------------------
* updates ownership
* Merge pull request `#11 <https://github.com/fetchrobotics/fetch_tools/issues/11>`_ from fetchrobotics/rctoris-patch-1
  Adds audio group to new users
* Update create_account.py
* Merge pull request `#10 <https://github.com/fetchrobotics/fetch_tools/issues/10>`_ from alexhenning/better-default-build
  No longer defaults to debug builds
* No longer defaults to debug builds
  Defaulting to debug was a mistake, instead, this provides an option that
  makes it easy to change the build type to debug (or anything else) with
  tab completion.
* Merge pull request `#9 <https://github.com/fetchrobotics/fetch_tools/issues/9>`_ from mehwang/more_hardware_info
  Expand hardware info retrieval and add read_board to debug_snapshot
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
*  Added three commands:
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

# Executing Tests

Run tests using `pytest` (`pip install pytest`). Always run tests from the root directory.

```
# A single test module
$ py.test tests/test_leaf_nodes.py
# A single test
$ py.test tests/test_leaf_nodes.py -k test_CheckGPS
```
Expect
```
$ py.test tests/test_leaf_nodes.py
============================= test session starts =============================
..............
==================== __ passed, _ warning in __s (_:_:__) =====================
```
# Two points interpolation PY
Calculate trajectory(pose, velocity, accerelation) between two points with constraints.

## Files
- two_point_interpolation_constant_acc: calculate trajectory with constant acc
- two_point_interpolation_test_constant_acc: test script for above.
- two_point_interpolation_constant_jerk: todo. calculate trajectory with constant jerk
- two_point_interpolation_test_constant_jerk: todo. test script for above.

## Example result
### Constant ACC
#### case 0: not reach velocity limit
constraints: 
- p0 = 10, pe = 170.0
- v0 = -5, ve = 1
- amax = 1, vmax = 100
- t0 = 10

![alt text](images/acc_constant_0.png)

#### case 1: reach velocity limit
constraints: 
- p0 = 50, pe = -90.0
- v0 = -5, ve = 1
- amax = 1, vmax = 10
- t0 = 0

![alt text](images/acc_constant_1.png)

=== Performance comparisons ===
This section discusses the performance implications of using quaternions versus other methods (axis/angle or rotation matrices) to perform rotations in 3D.

==== Results ====

{| class="wikitable"
|+ Storage requirements
|-
!                 Method                        !! Storage
|-
| [[Rotation matrix]]                           ||     9
|-
|    [[Quaternion]]                             ||     3 or 4 (see below)
|-
| [[Axis-angle representation|Angle/axis]] ||    3 or 4 (see below)
|}

Only three of the quaternion components are independent, as a rotation is represented by a unit quaternion.  For further calculation one usually needs all four elements, so all calculations would suffer additional expense from recovering the fourth component.  Likewise, angle/axis can be stored in a three-component vector by multiplying the unit direction by the angle (or a function thereof), but this comes at additional computational cost when using it for calculations.

{| class="wikitable"
|+ Performance comparison of rotation chaining operations
|-
!                 Method                !! # multiplies !! # add/subtracts !! total operations
|-
| Rotation matrices ||       27     ||       18        || 45
|-
|              Quaternions          ||       16     ||       12        || 28
|}

{| class="wikitable"
|+ Performance comparison of vector rotating operations<ref>Eberly, D., Rotation Representations and performance issues</ref><ref>https://bitbucket.org/eigen/eigen/src/4111270ba6e10882f8a8c571dbb660c8e9150138/Eigen/src/Geometry/Quaternion.h?at=default&fileviewer=file-view-default#Quaternion.h-469</ref>

|-
!         Method      !! # multiplies !! # add/subtracts !! # sin/cos !! total operations
|-
| Rotation matrix ||       9      ||        6       || 0         || 15
|-
|    Quaternions *  ||      21 ( = 12 + 9 )     ||       18 ( = 12 + 6 )     || 0         || 39 ( = 24 + 15 )
|-
| Angle/axis || 18 || 12 || 2     || 30
|}

<nowiki>*</nowiki> Note: Quaternions requires an implicit conversion to a rotation-like matrix (12 multiplies and 12 add/subtracts), which levels the following vectors rotating cost with the rotation matrix method
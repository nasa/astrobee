# Marker information

See points_world.csv for ground truth positions of registration points.

 1: M01: Marker / visualeyez pattern
 2: M02: Marker / visualeyez pattern
 3: M03: Marker / visualeyez pattern
 4: M04: Marker / visualeyez pattern
 5: M05: Marker
 6: M06: Marker
 7: M07: Marker
 8: M08: Marker
 9: M09: Marker
10: M10: Marker
11: M11: Marker
12: M12: Marker
13: C01: Room roof corner (see below)
14: C02: Room roof corner (see below)
15: C03: Room roof corner (see below)
16: C04: Room roof corner (see below)

- BIRDS EYE -

    [wall]
C02---B----C03
 |          |
 |          |       (X)-----> +X
 |          |        |
 A    C     |        |
 |          |       +Y
 |          |
 |          |
 |          |
C01--------C04 <-- Maybe a bit low / occluded by gimbal
    [door]

STEP1 : Axis alignment

    Wall A is the plane through (M06, M07, M09)
    Wall B is the plane through (M01, M02, M03)
    Floor / roof plane C is A x B

STEP2 : Offset

    M05, M06 and M09 define the origin
    X: (M05[X]+M09[X])/2
    Y: (M05[Y])
    Z: (M06[X]+M09[X])/2


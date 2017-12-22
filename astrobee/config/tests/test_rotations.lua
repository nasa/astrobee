-- required package is one directory up
package.path = package.path .. ";../?.lua;../?.config"

require "rotations"
require "common_lua"

-- Need to declare matrix locally
local matrix = require 'matrix'

c30 = math.cos(math.rad(30))
s30 = math.sin(math.rad(30))
rx90_ref = matrix{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}
rx30_ref = matrix{{1, 0, 0}, {0, c30, -s30}, {0, s30, c30}}
ry90_ref = matrix{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}
ry30_ref = matrix{{c30, 0, s30}, {0, 1, 0}, {-s30, 0, c30}}
rz90_ref = matrix{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}
rz30_ref = matrix{{c30, -s30, 0}, {s30, c30, 0}, {0, 0, 1}}
ridentity = matrix{
  {1.0, 0.0, 0.0},
  {0.0, 1.0, 0.0},
  {0.0, 0.0, 1.0}
}

function test_rot_matrix_functions()

  m1 = matrix{{0.0, 1.0}, {2.0, 3.0}}
  m2 = matrix{{-1E-9, 1.0+1E-9}, {2.0, 3.0}}
  if not matrix_equal(m1, m2) then return false end

  m3 = matrix{{0.0, 1.0}, {2.1, 3.0}}
  if matrix_equal(m1, m3) then return false end

  m4 = matrix{{0.0, 1.0}, {2.0, 3.0}, {4.0, 5.0}}
  if matrix_equal(m1, m4) then return false end
  print "Test matrix_equal(): PASS"

  rx90_test = rot_matrix(math.rad(90), 'x')
  if not matrix_equal(rx90_ref, rx90_test) then
    print("Test rx90 FAILED!")
    return false
  end

  rx30_test = rot_matrix(math.rad(30), 'x')
  if not matrix_equal(rx30_ref, rx30_test) then
    print("Test rx30 FAILED!")
    return false
  end

  ry90_test = rot_matrix(math.rad(90), 'y')
  if not matrix_equal(ry90_ref, ry90_test) then
    print("Test ry90 FAILED!")
    return false
  end

  ry30_test = rot_matrix(math.rad(30), 'y')
  if not matrix_equal(ry30_ref, ry30_test) then
    print("Test ry30 FAILED!")
    return false
  end

  rz90_test = rot_matrix(math.rad(90), 'z')
  if not matrix_equal(rz90_ref, rz90_test) then
    print("Test rz90 FAILED!")
    return false
  end

  rz30_test = rot_matrix(math.rad(30), 'z')
  if not matrix_equal(rz30_ref, rz30_test) then
    print("Test rz30 FAILED!")
    return false
  end

  print "Test rot_matrix(): PASS"

  return true
end

function test_euler_angles()
  m1 = rot_matrix_from_euler(0, 0, 0)
  if not matrix_equal(ridentity, m1) then
    print("Test rot_matrix_from_euler(0, 0, 0) FAILED!")
    return false
  end

  m2 = rot_matrix_from_euler(0.0, 0.0, math.pi/2.0)
  if not matrix_equal(rx90_ref, m2) then
    print("Test rot_matrix_from_euler(0.0, 0.0, math.pi/2.0) FAILED!")
    return false
  end

  m3 = rot_matrix_from_euler(0.0, 0.0, math.pi/6.0)
  if not matrix_equal(rx30_ref, m3) then
    print("Test rot_matrix_from_euler(0.0, 0.0, math.pi/6.0) FAILED!")
    return false
  end

  m4 = rot_matrix_from_euler(0.0, math.pi/2.0, 0.0)
  if not matrix_equal(ry90_ref, m4) then
    print("Test rot_matrix_from_euler(0.0, math.pi/2.0, 0.0) FAILED!")
    return false
  end

  m5 = rot_matrix_from_euler(0.0, math.pi/6.0, 0.0)
  if not matrix_equal(ry30_ref, m5) then
    print("Test rot_matrix_from_euler(0.0, math.pi/6.0, 0.0) FAILED!")
    return false
  end

  m6 = rot_matrix_from_euler(math.pi/2.0, 0.0, 0.0)
  if not matrix_equal(rz90_ref, m6) then
    print("Test rot_matrix_from_euler(math.pi/2.0, 0.0, 0.0) FAILED!")
    return false
  end

  m7 = rot_matrix_from_euler(math.pi/6.0, 0.0, 0.0)
  if not matrix_equal(rz30_ref, m7) then
    print("Test rot_matrix_from_euler(math.pi/6.0, 0.0, 0.0) FAILED!")
    return false
  end

  m8 = rot_matrix_from_euler(math.pi/6.0, math.pi/2.0, math.pi/6.0)
  mref = matrix.mul(rz30_ref, matrix.mul(ry90_ref, rx30_ref))
  if not matrix_equal(mref, m8) then
    print("Test rot_matrix_from_euler(math.pi/6.0, math.pi/2.0, math.pi/6.0) FAILED!")
    return false
  end

  m9 = rot_matrix_from_euler(math.pi/6.0, math.pi/6.0, math.pi/2.0)
  mref = matrix.mul(rz30_ref, matrix.mul(ry30_ref, rx90_ref))
  if not matrix_equal(mref, m9) then
    print("Test rot_matrix_from_euler(math.pi/6.0, math.pi/6.0, math.pi/2.0) FAILED!")
    return false
  end

  print "Test rot_matrix_from_euler: PASS"

  return true
end

function test_conversions()
  counter = 0
  for alpha=0,360,15 do
    rz = rot_matrix_z(math.rad(alpha))
    for beta=0,180,45 do
      ry = rot_matrix_y(math.rad(beta))
      rzy = matrix.mul(rz, ry)
      for gamma=-180,180,45 do
        counter = counter + 1
        rx = rot_matrix_x(math.rad(gamma))
        rzyx = matrix.mul(rzy, rx)

        reuler = euler2rmat(math.rad(alpha), math.rad(beta), math.rad(gamma))
        if not matrix_equal(rzyx, reuler) then
          print("Test Euler->RotMatrix failed for a="..alpha..", b="..beta..", g="..gamma)
          return false
        end

        q = rmat2quat(rzyx)
        rquat = quat2rmat(q)
        if not matrix_equal(rzyx, rquat) then
          print("Test quat --> rmat --> quat failed for  a="..alpha..", b="..beta..", g="..gamma)
          return false
        end
      end
    end
  end
  print("Test conversions (" .. counter .. " combinations): PASS")
  return true
end

print( test_rot_matrix_functions() )
print( test_euler_angles() )
print( test_conversions() )
--[[
print("rx30")
print(rx30_ref)
printquat(rmat2quat(rx30_ref))
print()
print("ry30")
print(ry30_ref)
printquat(rmat2quat(ry30_ref))
print()
print("rx30")
print(rz30_ref)
printquat(rmat2quat(rz30_ref))
print()
--]]

-- Useful tools to do some checks:
-- http://www.andre-gaschler.com/rotationconverter/

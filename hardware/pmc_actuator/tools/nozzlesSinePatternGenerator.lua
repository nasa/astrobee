-- Generates a sequence for PMC tests

-- number of PMCs
numPmcs = 2

-- duration of the generated sequence (seconds)
duration = 6

-- frame rate at wich commands are generated
rate = 5

-- speed of the impeller (fixed over the sequence)
speed = 207

-- number of nozzles on each PMCs
numNozzles = 6

-- min and max opening of the nozzles
minPosition = 5
maxPosition = 250

-- range of servo position
positionRange = maxPosition-minPosition

-- initial time
time = 0.0

-- offset the patterns
use_phase = true

-- time step
ts = 1.0 / rate

-- sinusoid power (<1 -> sharper peaks / >1 -> steeper slopes)
power = 0.8

-- modified sine wave
function wave(time, phase)
  sine = math.sin(2.0*time*math.pi/duration+phase)
  if sine < 0.0 then sign = -1.0 else sign = 1.0 end
  return 0.5 + 0.5 * sign * (1-math.pow(1-math.abs(sine), power))
end

-- Write first spin up
if time > 1E-6 then
  io.write("0.0 ")
  for p = 0,numPmcs-1 do
    io.write(speed .. " ")
    for n = 0,numNozzles-1 do
      io.write("0 ")
    end
  end
  io.write("\n")
end

-- main loop generatng a sequence of commands
for i = 0,rate*duration do
  io.write(string.format("%.3f ", time))
  for p = 0,numPmcs-1 do
    io.write(speed .. " ")
    for n = 0,numNozzles-1 do
      if use_phase then
        phase = n*2.0*math.pi/numNozzles
      else
        phase = 0.0
      end
      position = wave(time, phase)*positionRange+minPosition
      io.write(string.format("%d ", math.floor(position+0.5)))
    end
  end
  time = time+ts
  io.write("\n")
end

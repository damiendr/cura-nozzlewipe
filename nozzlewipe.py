#Name: Nozzle Wipe
#Info: Wipe the nozzle before and after retracting. This plugin requires a non-zero Z-hop while retracting (in Expert settings) to function.
#Depend: GCode
#Type: postprocess
#Param: takeoff_dist(float:10.0) Takeoff wipe distance (mm)
#Param: landing_dist(float:10.0) Landing wipe distance (mm)
#Param: z_hop_per_mm(float:0.015) Min. Z-hop per mm travelled (mm)
#Param: z_relief(float:0.05) Hover height during wiping (mm)
#Param: same_type(bool:False) Don't wipe across section boundaries

if __name__ == "__main__":
    takeoff_dist = 5.0
    landing_dist = 5.0
    z_relief = 0.05
    z_hop_per_mm = 0.015
    same_type = False

import sys
import math
from collections import namedtuple


GCode = namedtuple("GCode", "cmd args comment")

def parse_line(line):
    lineparts = line.split(";", 1)
    comment = ""
    if len(lineparts) == 2:
        comment = lineparts[1].rstrip()
    cmdparts = lineparts[0].strip().split(" ")
    cmd = cmdparts[0]
    args = cmdparts[1:]
    argdict = {arg[0]:arg[1:] for arg in args}
    return GCode(cmd, argdict, comment)


def distance(pos1, pos2):
    return math.sqrt(
        (float(pos1.args["X"]) - float(pos2.args["X"]))**2 +
        (float(pos1.args["Y"]) - float(pos2.args["Y"]))**2)


def extract_context(lines, count):
    moves = []
    z_pos = None
    collect = True
    for line in lines:
        gcode = parse_line(line)
        if same_type and "TYPE:" in gcode.comment:
            collect = False
        if "Z" in gcode.args:
            z_pos = float(gcode.args["Z"])
            collect = False
        if collect and "X" in gcode.args and "Y" in gcode.args:
            moves.append(gcode)
        if len(moves) == count:
            collect = False
        if not collect and z_pos is not None:
            break

    return moves, z_pos


def track_dist(moves):
    dist = 0.0
    steps = iter(moves)
    last_step = next(steps)
    yield last_step, dist
    for step in steps:
        delta = distance(step, last_step)
        dist += delta
        yield step, dist



def back_and_forth(seq):
    forth = list(seq)
    while True:
        for o in forth:
            yield o
        for o in reversed(forth):
            yield o


def make_hop(lines, i, retract, up, move, comment, down, extrude, after):

    yield GCode("", {}, "TYPE:Z-WIPE")

    moves_before, last_z = extract_context(lines[i-7::-1], 20)
    dist_before = 0.0
    for step, dist_before in track_dist(moves_before): pass

    # Raise the nozzle a bit to avoid skinking into the surface
    # while we retract:
    if last_z is not None:
        yield GCode("G0", {"Z":last_z + z_relief}, "raise a bit")

    E_down = float(extrude.args["E"])
    E_up = float(retract.args["E"])

    # Wipe the nozzle by retracing the last few steps without extruding:
    F = retract.args["F"]
    if dist_before > 0.0 and takeoff_dist > 0.0:
        retraction_steps = []
        for step, dist in track_dist(back_and_forth(moves_before)):
            E = E_down + (E_up - E_down) * min(1.0, dist/takeoff_dist)
            gcode = GCode("G1",
                {"X":step.args["X"],
                "Y":step.args["Y"],
                "E":E,
                "F":F}, "retract %.2f" % dist)
            yield gcode
            retraction_steps.append(gcode)
            if dist >= takeoff_dist: break

        # Retraction does not stop the flow immediately.
        # Retrace again, back to the start:
        for step in reversed(retraction_steps):
            del step.args["E"]
            step.args["F"] = F
            yield step
    else:
        # Not enough context moves, let's just retract in place:
        yield retract

    # Hop to the next location, increasing Z gradually:
    if moves_before:
        hop_dist = distance(moves_before[0], move)
    else:
        hop_dist = 0.0
    z_next = float(down.args["Z"])
    z_up = max(z_next + z_hop_per_mm * hop_dist, float(up.args["Z"]))

    hop = GCode("G0",
        {"X":move.args["X"], "Y":move.args["Y"], "Z":z_up, "F":9000}, "hop")
    yield hop

    # We're there. Lower the nozzle again:
    yield GCode("G0", {"Z":float(down.args["Z"]) + z_relief}, "lower a bit")

    # The nozzle may have drooled a bit during the move. Wipe it again
    # upon landing:
    moves_after, _ = extract_context(lines[i:], 20)
    dist_after = 0.0
    for step, dist_after in track_dist(moves_after): pass

    if dist_after > 0.0 and landing_dist > 0.0:
        E = E_up
        F = extrude.args["F"]
        last_step = moves_after[0]
        landing_steps = []
        for step, dist in track_dist(back_and_forth(moves_after)):
            landing_steps.append(step)
            yield GCode("G1",
                {"X":step.args["X"], "Y":step.args["Y"], "F":3000},
                "wipe %.2f" % dist)
            last_step = step
            if dist >= landing_dist: break

        for step, dist in track_dist(reversed(landing_steps)):
            E = E_up + (E_down - E_up) * min(1.0, dist/landing_dist)
            yield GCode("G1",
                {"X":step.args["X"], "Y":step.args["Y"], "E":E, "F":F},
                "extrude %.2f" % dist)
            last_step = step
        yield comment
        yield down
    else:
        yield comment
        yield down
        yield extrude # Not enough context, extrude in place


def gcode_to_string(gcode):
    parts = []
    if gcode.cmd: parts.append(gcode.cmd)
    for pair in gcode.args.items():
        parts.append("%s%s" % pair)
    if gcode.comment:
        parts.append(";%s" % gcode.comment)
    return " ".join(parts)

# Example match:
# G1 F3000 E929.45693
# G1 Z33.000
# G0 F9000 X73.472 Y27.640
# ;TYPE:WALL-OUTER
# G1 Z32.700
# G1 F3000 E934.45693
hop_pattern = [
    ("G1", ("FE",)),
    ("G1", ("Z",)),
    ("G0", ("FXY","FXYZ")),
    ("", ("",)),
    ("G1", ("Z",)),
    ("G1", ("FE",)),
    ("G1", ("FEXY",)),
]

def match(gcode, pattern, idx):
    cmd, args = pattern[idx]
    return gcode.cmd == cmd and any(set(gcode.args.keys()) == set(expected)
                                    for expected in args)


def process(lines):
    buf = []
    for i, line in enumerate(lines):
        gcode = parse_line(line)

        if gcode.cmd == "G91":
            raise Exception("Relative mode is not supported")

        if len(buf) == 0:
            if match(gcode, hop_pattern, 0):
                buf.append(gcode)
                continue

        if len(buf) > 0:
            if match(gcode, hop_pattern, len(buf)):
                buf.append(gcode)

                if len(buf) == len(hop_pattern):
#                    try:
                    for new_code in make_hop(lines, i, *buf):
                        yield new_code
                    # except Exception as e:
                    #     for cmd in buf:
                    #         yield cmd
                    # Done:
                    buf = []

                continue
            else:
                # Not all the lines matched. Empty the buffer:
                for prev_gcode in buf: yield(prev_gcode)
                buf = []

        yield gcode

    # Empty anything left in the buffer:
    for gcode in buf: yield gcode
    return


if __name__ == "__main__":
    with open(sys.argv[1], "r") as f:
        lines = f.readlines()

    for gcode in process(lines):
        print gcode_to_string(gcode)

else:
    with open(filename, "r") as f:
        lines = f.readlines()

    with open(filename, "w") as f:
        for gcode in process(lines):
            f.write(gcode_to_string(gcode))
            f.write("\n")

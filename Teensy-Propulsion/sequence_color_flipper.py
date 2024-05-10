string = """
    new StaticAction(SOLAR_LEFT_ON),
    new MoveAction(VectorOriented(0.70f, 1.77, 0.0f), false, false, true, true),
    new MoveAction(VectorOriented(0.70f, 1.77, -0.9*PI/4), true, false, true, true),
    new StaticAction(SOLAR_LEFT_OFF,true),
"""

splitted = string.split("VectorOriented(")
result = splitted[0]
for s in splitted[1:]:
    entries = s.split(")", 1)[0].split(",")
    new_entries = "3.0f - " + entries[0] + ", " + entries[1] + ", " + "normalizeAngle(PI - " + entries[2] + ")"
    result += new_entries
    result += ")".join(s.split(")")[1:])

result = "SOLAR_R_TMP".join(result.split("SOLAR_LEFT"))
result = "SOLAR_LEFT".join(result.split("SOLAR_RIGHT"))
result = "SOLAR_RIGHT".join(result.split("SOLAR_R_TMP"))

print(result)

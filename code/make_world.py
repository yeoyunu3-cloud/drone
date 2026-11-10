"""course.py를 읽어 Gazebo 도시 맵(~/drone/worlds/city.sdf)을 만든다.

course.py에서 건물이나 게이트 위치를 바꿨으면 이걸 다시 실행한다:
    python make_world.py

PX4는 자기 폴더(PX4-Autopilot/Tools/simulation/gz/worlds)에서만 맵을 찾으므로
거기에 city.sdf 바로가기(심볼릭 링크)도 만들어 둔다.
"""

import math
from pathlib import Path

import course

HOME = Path.home() / "drone"
PX4_WORLDS = HOME / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "worlds"
WORLD_NAME = "city"
OUTPUT = HOME / "worlds" / f"{WORLD_NAME}.sdf"

PERSON_URI = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"

# 건물 색 (회색, 베이지, 푸른 회색). 게이트의 주황색과 헷갈리지 않는 색만 쓴다
BUILDING_COLORS = ["0.55 0.55 0.58", "0.70 0.66 0.58", "0.45 0.50 0.58", "0.62 0.62 0.62"]
GATE_COLOR = "1.0 0.35 0.0"
GATE_GLOW = "0.6 0.2 0.0"  # 그림자 속에서도 주황색으로 보이게 스스로 빛나는 정도


def box_part(name, x, y, z, sx, sy, sz, color, glow=None):
    """보이는 모양(visual)과 부딪히는 모양(collision)이 같은 상자 하나."""
    emissive = f"<emissive>{glow} 1</emissive>" if glow else ""
    geometry = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    return f"""
        <collision name="{name}_collision">
          <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
          {geometry}
        </collision>
        <visual name="{name}_visual">
          <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
          {geometry}
          <material>
            <ambient>{color} 1</ambient>
            <diffuse>{color} 1</diffuse>
            {emissive}
          </material>
        </visual>"""


def static_model(name, x, y, yaw, parts):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
      <link name="link">{parts}
      </link>
    </model>"""


def building_model(box, color):
    return static_model(box.name, box.x, box.y, box.yaw,
                        box_part("body", 0, 0, box.height / 2, box.size_x, box.size_y, box.height, color))


def gate_model(gate):
    # 게이트 기준 좌표: x는 지나가는 방향, y는 옆. 막대 위치는 course.py가 정한다
    parts = "".join(box_part(*part, GATE_COLOR, GATE_GLOW) for part in gate.parts())
    return static_model(gate.name, gate.x, gate.y, gate.yaw, parts)


def person_model():
    x, y = course.PERSON
    yaw = math.atan2(course.START[1] - y, course.START[0] - x)  # 출발점을 바라보게
    return f"""
    <include>
      <name>person</name>
      <uri>{PERSON_URI}</uri>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
    </include>"""


VIEW_CAMERA_NAME = "view_camera"
VIEW_CAMERA_TOPIC = "/view_camera/image"
VIEW_CAMERA_SIZE = (960, 720)


def view_camera_model():
    """조종 창의 추격 시점·위에서 보기용 카메라. 모양이 없어서 드론 카메라에는 안 보인다.
    view_camera.py가 1초에 30번 드론 근처로 옮긴다."""
    w, h = VIEW_CAMERA_SIZE
    return f"""
    <model name="{VIEW_CAMERA_NAME}">
      <static>true</static>
      <pose>-5 0 3 0 0.45 0</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <topic>{VIEW_CAMERA_TOPIC}</topic>
          <camera>
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>300</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""


def build_world():
    # PX4 기본 맵(땅, 햇빛, GPS 기준 위치)을 바탕으로 이름만 바꾸고 모델을 더한다
    template = (PX4_WORLDS / "default.sdf").read_text(encoding="utf-8-sig")
    models = "".join(building_model(b, BUILDING_COLORS[i % len(BUILDING_COLORS)])
                     for i, b in enumerate(course.BUILDINGS))
    models += "".join(gate_model(g) for g in course.GATES)
    models += person_model()
    models += view_camera_model()
    world = template.replace('<world name="default">', f'<world name="{WORLD_NAME}">', 1)
    return world.replace("</world>", models + "\n  </world>", 1)


def main():
    problems = course.check_course()
    if problems:
        raise SystemExit("코스 경로에 문제가 있어서 맵을 만들지 않음:\n  " + "\n  ".join(problems))

    OUTPUT.parent.mkdir(exist_ok=True)
    OUTPUT.write_text(build_world(), encoding="utf-8")
    print(f"맵 저장: {OUTPUT}")

    link = PX4_WORLDS / OUTPUT.name
    if not link.is_symlink():
        link.symlink_to(OUTPUT)
        print(f"PX4 맵 폴더에 바로가기 만듦: {link}")


if __name__ == "__main__":
    main()

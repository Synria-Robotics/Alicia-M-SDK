URDF_LIMIT = {
    "ALICIA_D": [
        {"jointName": "Joint1", "lower": -157.5, "upper": 157.5},
        {"jointName": "Joint2", "lower": -100.2, "upper": 100.2},
        {"jointName": "Joint3", "lower": -34.3, "upper": 126.0},
        {"jointName": "Joint4", "lower": -159.8, "upper": 159.8},
        {"jointName": "Joint5", "lower": -89.9, "upper": 89.9},
        {"jointName": "Joint6", "lower": -179.9, "upper": 179.9},
    ],
    "ALICIA_M": [
        {"jointName": "Joint1", "lower": -157.5, "upper": 157.5},
        {"jointName": "Joint2", "lower": -179.9, "upper": 0},
        {"jointName": "Joint3", "lower": -179.9, "upper": 0},
        {"jointName": "Joint4", "lower": -89.9, "upper": 89.9},
        {"jointName": "Joint5", "lower": -89.9, "upper": 89.9},
        {"jointName": "Joint6", "lower": -157.5, "upper": 157.5},
    ],
}


REVERSED_JOINT_INDEXES = {3, 5}
PROPORTIONAL_JOINT_INDEXES = {2}
NEGATED_INPUT_JOINT_INDEXES = {2}


def clamp(value: float, lower: float, upper: float) -> float:
    """
    将值限制在指定区间内。
    """
    return max(lower, min(value, upper))


def map_joint_value(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    """
    映射关节值

    :param value: 关节值
    :param src_min: 原关节最小值
    :param src_max: 原关节最大值
    :param dst_min: 目标关节最小值
    :param dst_max: 目标关节最大值
    :return: 映射后的关节值
    """
    return ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min) + dst_min


def map_joint_value_with_m_limit(
    value: float,
    dst_min: float,
    dst_max: float,
    reverse: bool = False,
    align_to_center: bool = True,
) -> float:
    """
    保持 D/M 角度 1:1，并限制在 M 的限位内。

    规则：
    - align_to_center=True 时，D 的 0 度 -> M 的区间中点
    - align_to_center=False 时，D 的 0 度 -> M 的 0 度
    - M 的 1 度就是 D 的 1 度
    - reverse=True 时目标方向翻转
    - 最终输出不会超过 M 的限位
    """
    dst_origin = (dst_min + dst_max) / 2 if align_to_center else 0.0
    direction = -1.0 if reverse else 1.0
    mapped = direction * value + dst_origin
    return clamp(mapped, min(dst_min, dst_max), max(dst_min, dst_max))


def convert_joints_deg_from_alicia_d_to_alicia_m(joints_deg: list[float]) -> list[float]:
    """
    将灵动关节值映射为云擎关节值

    注意：
    - 该函数会返回一个新的列表
    - 以 D 的 0 度对齐到 M 的区间中点
    - M 的 1 度就是 D 的 1 度，不做区间比例缩放
    - Python 下标 3、5（即第 4、6 个关节）采用反向映射
    - 第 3 个关节（下标 2）特殊处理：先取负，再按 D[-126.0, 34.3] -> M[-179.9, 0] 做比例映射

    :param joints_deg: 灵动关节值
    :return: 云擎关节值
    """
    result = []

    for i, joint in enumerate(joints_deg):
        if i in PROPORTIONAL_JOINT_INDEXES:
            if i in NEGATED_INPUT_JOINT_INDEXES:
                joint = -joint
                src_min = -URDF_LIMIT["ALICIA_D"][i]["upper"]
                src_max = -URDF_LIMIT["ALICIA_D"][i]["lower"]
            else:
                src_min = URDF_LIMIT["ALICIA_D"][i]["lower"]
                src_max = URDF_LIMIT["ALICIA_D"][i]["upper"]
            dst_min = URDF_LIMIT["ALICIA_M"][i]["lower"]
            dst_max = URDF_LIMIT["ALICIA_M"][i]["upper"]
            mapped = map_joint_value(
                clamp(joint, src_min, src_max),
                src_min,
                src_max,
                dst_min,
                dst_max,
            )
            mapped = clamp(mapped, min(dst_min, dst_max), max(dst_min, dst_max))
        else:
            mapped = map_joint_value_with_m_limit(
                joint,
                URDF_LIMIT["ALICIA_M"][i]["lower"],
                URDF_LIMIT["ALICIA_M"][i]["upper"],
                reverse=i in REVERSED_JOINT_INDEXES,
            )
        result.append(mapped)

    return result


if __name__ == "__main__":
    sample_joints = [0, 0, 0, 0, 0, 0]
    converted = convert_joints_deg_from_alicia_d_to_alicia_m(sample_joints)
    print("输入关节值:", sample_joints)
    print("映射后关节值:", converted)

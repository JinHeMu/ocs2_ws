import numpy as np
from scipy.ndimage import distance_transform_edt
import mujoco


def is_static_body(model, body_id):
    """
    判断该 body 及其祖先是否都没有 joint。
    有 joint 的 body 通常属于机器人/动态物体，不应写入静态 ESDF。
    """
    while body_id != 0:  # 0 是 world body
        if model.body_jntnum[body_id] > 0:
            return False
        body_id = model.body_parentid[body_id]
    return True


def voxelize_oriented_box(occ, origin, voxel_size, center, R, half_size):
    """
    把一个 MuJoCo box geom 体素化到 occupancy grid。
    center: world position, shape (3,)
    R: local-to-world rotation matrix, shape (3, 3)
    half_size: box half extents, shape (3,)
    """
    # 计算旋转 box 的 world AABB，用于裁剪体素范围
    signs = np.array([
        [-1, -1, -1],
        [-1, -1,  1],
        [-1,  1, -1],
        [-1,  1,  1],
        [ 1, -1, -1],
        [ 1, -1,  1],
        [ 1,  1, -1],
        [ 1,  1,  1],
    ], dtype=float)

    corners_local = signs * half_size
    corners_world = center + corners_local @ R.T

    box_min = corners_world.min(axis=0)
    box_max = corners_world.max(axis=0)

    i0 = np.floor((box_min - origin) / voxel_size).astype(int) - 1
    i1 = np.ceil((box_max - origin) / voxel_size).astype(int) + 1

    i0 = np.maximum(i0, 0)
    i1 = np.minimum(i1, np.array(occ.shape))

    if np.any(i0 >= i1):
        return

    xs = origin[0] + (np.arange(i0[0], i1[0]) + 0.5) * voxel_size
    ys = origin[1] + (np.arange(i0[1], i1[1]) + 0.5) * voxel_size
    zs = origin[2] + (np.arange(i0[2], i1[2]) + 0.5) * voxel_size

    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")
    pts = np.stack([X, Y, Z], axis=-1).reshape(-1, 3)

    # world -> box local
    local = (pts - center) @ R

    inside = np.all(np.abs(local) <= (half_size + 0.5 * voxel_size), axis=1)
    inside = inside.reshape(X.shape)

    occ[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]] |= inside


def build_esdf_from_mjcf(
    xml_path,
    voxel_size=0.02,
    bounds_min=(-3.3, -3.3, 0.0),
    bounds_max=(3.3, 3.3, 1.3),
    include_floor=False,
    output_path="scene_esdf.npz",
):
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 计算初始状态下所有 geom 的 world 位姿
    mujoco.mj_forward(model, data)

    origin = np.array(bounds_min, dtype=float)
    bounds_max = np.array(bounds_max, dtype=float)
    grid_shape = np.ceil((bounds_max - origin) / voxel_size).astype(int)

    occ = np.zeros(grid_shape, dtype=bool)

    BOX = int(mujoco.mjtGeom.mjGEOM_BOX)
    PLANE = int(mujoco.mjtGeom.mjGEOM_PLANE)

    used_geoms = []

    for gid in range(model.ngeom):
        geom_type = int(model.geom_type[gid])
        body_id = int(model.geom_bodyid[gid])

        # 跳过机器人、关节体、动态体
        if not is_static_body(model, body_id):
            continue

        # 跳过不参与碰撞的 geom
        if model.geom_contype[gid] == 0 and model.geom_conaffinity[gid] == 0:
            continue

        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if name is None:
            name = f"geom_{gid}"

        # floor 通常不作为移动规划障碍
        if geom_type == PLANE:
            if include_floor:
                occ[:, :, 0] = True
                used_geoms.append(name)
            continue

        # 这里先处理 box；你的场景主要就是 box
        if geom_type == BOX:
            center = data.geom_xpos[gid].copy()
            R = data.geom_xmat[gid].reshape(3, 3).copy()
            half_size = model.geom_size[gid, :3].copy()

            voxelize_oriented_box(
                occ=occ,
                origin=origin,
                voxel_size=voxel_size,
                center=center,
                R=R,
                half_size=half_size,
            )
            used_geoms.append(name)
        else:
            print(f"[WARN] skip unsupported geom type: {name}, type={geom_type}")

    # ESDF：自由空间为正，障碍内部为负
    # distance_transform_edt 是精确欧氏距离变换。这里 sampling=voxel_size 会让结果单位为米。
    dist_outside = distance_transform_edt(~occ, sampling=voxel_size)
    dist_inside = distance_transform_edt(occ, sampling=voxel_size)

    esdf = dist_outside - dist_inside

    np.savez_compressed(
        output_path,
        esdf=esdf.astype(np.float32),
        occupancy=occ,
        origin=origin.astype(np.float32),
        voxel_size=np.float32(voxel_size),
        bounds_max=bounds_max.astype(np.float32),
        used_geoms=np.array(used_geoms),
    )

    print("Saved:", output_path)
    print("Grid shape:", grid_shape)
    print("Voxel size:", voxel_size)
    print("Used geoms:", used_geoms)

    return esdf, occ, origin, voxel_size


if __name__ == "__main__":
    build_esdf_from_mjcf(
        xml_path="/home/a/ocs2_tracer_jaka/src/tracer_jaka_mujoco/models/scene.xml",
        voxel_size=0.02,
        bounds_min=(-3.0, -3.0, 0.0),
        bounds_max=(3.0, 3.0, 1.3),
        include_floor=False,
        output_path="maps/tracer_jaka_zu5_scene_esdf.npz",
    )

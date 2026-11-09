import math
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from panda3d.core import (
    AmbientLight,
    CardMaker,
    DirectionalLight,
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    LPoint3,
    LQuaternion,
    Material,
    NodePath,
    Vec3,
    Vec4,
)


def create_smooth_loft(profile_fn, length, rings=60, slices=56):
  """Generuje zaawansowany segment z niezależnym rzeźbieniem poszczególnych komór mięśniowych."""
  vdata = GeomVertexData('limb', GeomVertexFormat.getV3n3(), Geom.UHStatic)
  vertex = GeomVertexWriter(vdata, 'vertex')
  normal = GeomVertexWriter(vdata, 'normal')
  tris = GeomTriangles(Geom.UHStatic)

  for r in range(rings + 1):
    u = r / rings
    rx, ry, q_front, v_med, m_add, dx, dy, dz = profile_fn(u)

    du = 0.002
    rx_n, ry_n, qf_n, vm_n, ma_n, dx_n, dy_n, dz_n = profile_fn(
        min(1.0, u + du)
    )
    drx_dz = (rx_n - rx) / (-du * length)
    dry_dz = (ry_n - ry) / (-du * length)

    for s in range(slices + 1):
      phi = 2.0 * math.pi * s / slices
      cp, sp = math.cos(phi), math.sin(phi)

      # 1. Czworogłowy - przód (sp > 0)
      quad_add = q_front * (sp**1.4) if sp > 0 else 0.0

      # 2. Vastus medialis (łezka) - przód-wewnątrz (sp > 0 i cp > 0)
      medialis_add = v_med * (sp * cp) if (sp > 0 and cp > 0) else 0.0

      # 3. Przywodziciele - wyłącznie strona wewnętrzna (cp > 0) w górnej partii
      adductor_add = m_add * (cp**1.3) if cp > 0 else 0.0

      eff_rx = rx + adductor_add + medialis_add * 0.7
      eff_ry = ry + quad_add + medialis_add * 0.5 + adductor_add * 0.3

      x = eff_rx * cp + dx
      y = eff_ry * sp + dy
      z = dz
      vertex.addData3(x, y, z)

      nx = cp / max(eff_rx, 1e-4)
      ny = sp / max(eff_ry, 1e-4)
      nz = -(drx_dz * cp + dry_dz * sp)
      ln = math.sqrt(nx * nx + ny * ny + nz * nz)
      normal.addData3(nx / ln, ny / ln, nz / ln)

  for r in range(rings):
    for s in range(slices):
      i1 = r * (slices + 1) + s
      i2 = i1 + 1
      i3 = (r + 1) * (slices + 1) + s
      i4 = i3 + 1
      tris.addVertices(i1, i3, i2)
      tris.addVertices(i2, i3, i4)

  geom = Geom(vdata)
  geom.addPrimitive(tris)
  node = GeomNode('limb_geom')
  node.addGeom(geom)
  return NodePath(node)


def create_knee_joint_core(radius=0.048, width=0.088):
  vdata = GeomVertexData('joint', GeomVertexFormat.getV3n3(), Geom.UHStatic)
  vertex = GeomVertexWriter(vdata, 'vertex')
  normal = GeomVertexWriter(vdata, 'normal')
  tris = GeomTriangles(Geom.UHStatic)

  rings, slices = 20, 32
  for r in range(rings + 1):
    theta = math.pi * r / rings
    st, ct = math.sin(theta), math.cos(theta)
    for s in range(slices + 1):
      phi = 2.0 * math.pi * s / slices
      cp, sp = math.cos(phi), math.sin(phi)

      x = 0.5 * width * ct
      y = radius * st * cp
      z = radius * st * sp
      vertex.addData3(x, y, z)

      nx = ct / (0.5 * width)
      ny = (st * cp) / radius
      nz = (st * sp) / radius
      ln = math.sqrt(nx * nx + ny * ny + nz * nz)
      normal.addData3(nx / ln, ny / ln, nz / ln)

  for r in range(rings):
    for s in range(slices):
      p1 = r * (slices + 1) + s
      p2 = p1 + 1
      p3 = (r + 1) * (slices + 1) + s
      p4 = p3 + 1
      tris.addVertices(p1, p3, p2)
      tris.addVertices(p2, p3, p4)

  geom = Geom(vdata)
  geom.addPrimitive(tris)
  node = GeomNode('knee_core_geom')
  node.addGeom(geom)
  return NodePath(node)


def create_compact_foot(length=0.24, width=0.088, height=0.072):
    """
    Anatomiczna stopa z płaską podeszwą, wyprofilowanym grzbietem,
    masywną zaokrągloną piętą i naturalnym łukiem palców.
    """
    vdata = GeomVertexData('foot', GeomVertexFormat.getV3n3(), Geom.UHStatic)
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    tris = GeomTriangles(Geom.UHStatic)

    n_len = 36
    n_ring = 32
    heel_ext = 0.038  # naturalne cofnięcie guza piętowego za oś obrotu

    for i in range(n_len + 1):
        u = i / n_len
        y = -heel_ext + u * (length + heel_ext)

        # 1. Zaokrąglenie skrajów: pięta z tyłu (u bliskie 0), łuk palców z przodu (u bliskie 1)
        cap = 1.0
        if u < 0.12:
            cap = math.sin((u / 0.12) * (math.pi / 2.0))
        elif u > 0.88:
            # Szerokie zaokrąglenie linii palców zamiast ostrego stożka
            cap = math.cos(((u - 0.88) / 0.12) * (math.pi / 2.0) * 0.75)

        # 2. Rozkład szerokości w płaszczyźnie poprzecznej
        if u < 0.22:
            # Guz piętowy
            w = width * (0.64 + 0.26 * (u / 0.22))
        elif u < 0.72:
            # Śródstopie rozszerzające się w stronę głów kości śródstopia
            prog = (u - 0.22) / 0.50
            w = width * (0.90 + 0.10 * prog)
        else:
            # Łagodne ścięcie ku palcom
            prog = (u - 0.72) / 0.28
            w = width * (1.0 - 0.25 * (prog ** 1.3))

        # 3. Profil wysokości i grzbietu stopy (opadanie od bloczka skokowego)
        if u < 0.25:
            # Pięta i okolice stawu skokowego
            h_top = height * (0.75 + 0.25 * (u / 0.25))
            arch = 0.0
        elif u < 0.75:
            # Grzbiet stopy płynnie opadający ku palcom
            prog = (u - 0.25) / 0.50
            h_top = height * (1.0 - 0.52 * prog)
            arch = 0.008 * math.sin(prog * math.pi)  # sklepienie przyśrodkowe
        else:
            # Palce stopy (spłaszczone)
            prog = (u - 0.75) / 0.25
            h_top = height * 0.48 * (1.0 - 0.40 * prog)
            arch = 0.001 * (1.0 - prog)

        eff_w = max(w * cap, 0.003)
        h_bottom = height * 0.28 * cap  # grubość podeszwy/poduszki

        for j in range(n_ring + 1):
            ang = 2.0 * math.pi * j / n_ring
            s_val = math.sin(ang)
            c_val = math.cos(ang)

            # Płaska podeszwa (s_val < 0) vs zaokrąglony grzbiet stopy (s_val >= 0)
            if s_val < 0:
                z = s_val * h_bottom + arch
                nz = s_val / max(h_bottom, 1e-4)
            else:
                z = s_val * (h_top * cap) + arch
                nz = s_val / max(h_top * cap, 1e-4)

            x = c_val * (eff_w / 2.0)
            nx = c_val / max(eff_w / 2.0, 1e-4)

            vertex.addData3(x, y, z)

            ln = math.sqrt(nx * nx + 0.04 + nz * nz)
            normal.addData3(nx / ln, 0.2 / ln, nz / ln)

    for i in range(n_len):
        for j in range(n_ring):
            p1 = i * (n_ring + 1) + j
            p2 = p1 + 1
            p3 = (i + 1) * (n_ring + 1) + j
            p4 = p3 + 1
            tris.addVertices(p1, p3, p2)
            tris.addVertices(p2, p3, p4)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode('foot_geom')
    node.addGeom(geom)
    return NodePath(node)


class RefinedLegViewer(ShowBase):

  def __init__(self):
    super().__init__()

    # Kamera
    self.disableMouse()
    self.cam_target = LPoint3(0.0, 0.0, 0.45)
    self.cam_dist = 2.1
    self.cam_yaw = 25.0
    self.cam_pitch = 10.0

    self.last_mouse_x = 0
    self.last_mouse_y = 0
    self.mouse_dragging = False

    self.accept('wheel_up', self.zoom, [-0.12])
    self.accept('wheel_down', self.zoom, [0.12])
    self.update_camera()

    # Oświetlenie
    alight = AmbientLight('alight')
    alight.setColor(Vec4(0.42, 0.42, 0.45, 1.0))
    self.render.setLight(self.render.attachNewNode(alight))

    key = DirectionalLight('key')
    key.setColor(Vec4(0.85, 0.83, 0.80, 1.0))
    np_key = self.render.attachNewNode(key)
    np_key.setHpr(42, -32, 0)
    self.render.setLight(np_key)

    fill = DirectionalLight('fill')
    fill.setColor(Vec4(0.35, 0.38, 0.42, 1.0))
    np_fill = self.render.attachNewNode(fill)
    np_fill.setHpr(-135, -20, 0)
    self.render.setLight(np_fill)

    # Podłoga
    cm = CardMaker('ground')
    cm.setFrame(-2, 2, -2, 2)
    ground = self.render.attachNewNode(cm.generate())
    ground.setP(-90)
    ground.setPos(0, 0, 0)
    ground.setColor(0.14, 0.15, 0.17, 1.0)

    # Materiał
    mat_leg = Material()
    mat_leg.setDiffuse(Vec4(0.88, 0.86, 0.83, 1.0))
    mat_leg.setSpecular(Vec4(0.20, 0.20, 0.20, 1.0))
    mat_leg.setShininess(24.0)

    # Parametry szkieletu
    self.L_THIGH = 0.44
    self.L_SHIN = 0.41
    self.L_FOOT = 0.23

    self.H_HIP = self.L_THIGH + self.L_SHIN + 0.04
    self.hip_joint = self.render.attachNewNode('hip_joint')
    self.hip_joint.setPos(0, 0, self.H_HIP)

    # UDO: gładkie ścięgno czworogłowego bez ząbka nad kolanem
    def thigh_profile(u):
      z = -u * self.L_THIGH

      # 1. Zaokrąglenie góry (u in [0, 0.10])
      cap_top = 1.0
      if u < 0.10:
        cap_top = math.sin((u / 0.10) * (math.pi / 2.0))

      # 2. Zaokrąglenie dołu do osi kolana (u in [0.90, 1.0]) - likwiduje ostry mankiet
      cap_bottom = 1.0
      if u > 0.90:
        cap_bottom = math.cos(((u - 0.90) / 0.10) * (math.pi / 2.0) * 0.5)

      cap = cap_top * cap_bottom

      # Współczynnik wygaszania mięśni przed samym stawem kolanowym (u > 0.82)
      fade_to_tendon = 1.0
      if u > 0.82:
        fade_to_tendon = math.cos(((u - 0.82) / 0.18) * (math.pi / 2.0))

      # Baza poprzeczna (Twoje rx = 0.075)
      taper_x = 1.0 - 0.38 * u
      lateral_quad = (
          0.010
          * math.sin(u * math.pi * 0.75)
          * (1.0 - 0.3 * u)
          * fade_to_tendon
      )
      condyle_support = 0.005 * (u**3)
      rx = (0.085 * taper_x + lateral_quad + condyle_support) * cap

      # Przekrój strzałkowy (ry) zbiegający się gładko do promienia przegubu (0.046)
      taper_y = 1.0 - 0.48 * u
      ry = (0.088 * taper_y) * cap

      # Brzusiec czworogłowego – płynnie wygaszany w ścięgno rzepki:
      quad_front = 0.026 * math.sin(u * math.pi * 0.85) * fade_to_tendon

      # Vastus medialis – skupiona łezka, która wygasa przed szczeliną stawu:
      vastus_med = (
          0.028 * math.exp(-(((u - 0.72) / 0.10) ** 2)) * fade_to_tendon
      )

      # Przywodziciele (góra, strona wewnętrzna)
      adductors = 0.024 * math.exp(-((u / 0.35) ** 2)) * (1.0 - u)

      dx = 0.0
      dy = 0.004 * math.sin(u * math.pi * 0.8) * fade_to_tendon
      return rx, ry, quad_front, vastus_med, adductors, dx, dy, z

    thigh = create_smooth_loft(thigh_profile, self.L_THIGH)
    thigh.reparentTo(self.hip_joint)
    thigh.setMaterial(mat_leg)
    thigh.setColor(Vec4(0.88, 0.86, 0.83, 1.0))

    # KOLANO
    self.knee_joint = self.hip_joint.attachNewNode('knee_joint')
    self.knee_joint.setPos(0, 0, -self.L_THIGH)

    knee_core = create_knee_joint_core(radius=0.046, width=0.086)
    knee_core.reparentTo(self.knee_joint)
    knee_core.setMaterial(mat_leg)
    knee_core.setColor(Vec4(0.85, 0.83, 0.80, 1.0))

    # GOLEŃ
    def shin_profile(u):
      z = -u * self.L_SHIN
      cap = 1.0
      if u > 0.94:
        cap = math.cos(((u - 0.94) / 0.06) * (math.pi / 2.0))

      calf_side = 0.014 * math.exp(-(((u - 0.25) / 0.22) ** 2))
      malleolus = 0.007 * (u**5)
      rx = (0.048 * (1.0 - 0.26 * u) + calf_side + malleolus) * cap

      calf_back = 0.016 * math.exp(-(((u - 0.25) / 0.20) ** 2))
      ry = (0.046 * (1.0 - 0.26 * u) + calf_back) * cap

      dx = 0.003 * math.sin(u * math.pi)
      dy = -calf_back * 0.55
      return rx, ry, 0.0, 0.0, 0.0, dx, dy, z

    shin = create_smooth_loft(shin_profile, self.L_SHIN)
    shin.reparentTo(self.knee_joint)
    shin.setMaterial(mat_leg)
    shin.setColor(Vec4(0.88, 0.86, 0.83, 1.0))

    # KOSTKA I STOPA
    self.ankle_joint = self.knee_joint.attachNewNode('ankle_joint')
    self.ankle_joint.setPos(0, 0, -self.L_SHIN)

    foot = create_compact_foot(length=self.L_FOOT)
    foot.reparentTo(self.ankle_joint)
    foot.setMaterial(mat_leg)
    foot.setColor(Vec4(0.88, 0.86, 0.83, 1.0))

    self.taskMgr.add(self.update_camera_task, 'CamTask')
    self.taskMgr.add(self.update_motion, 'MotionTask')

  def zoom(self, delta):
    self.cam_dist = max(0.8, min(6.0, self.cam_dist + delta))
    self.update_camera()

  def update_camera(self):
    ry = math.radians(self.cam_yaw)
    rp = math.radians(self.cam_pitch)
    cx = self.cam_dist * math.cos(rp) * math.sin(ry)
    cy = -self.cam_dist * math.cos(rp) * math.cos(ry)
    cz = self.cam_dist * math.sin(rp)
    self.camera.setPos(self.cam_target + Vec3(cx, cy, cz))
    self.camera.lookAt(self.cam_target)

  def update_camera_task(self, task):
    if not self.mouseWatcherNode.hasMouse():
      return Task.cont

    m_pos = self.mouseWatcherNode.getMouse()
    mx, my = m_pos.getX(), m_pos.getY()

    if self.mouseWatcherNode.isButtonDown('mouse1'):
      if self.mouse_dragging:
        dx = mx - self.last_mouse_x
        dy = my - self.last_mouse_y
        self.cam_yaw -= dx * 105.0
        self.cam_pitch = max(-40.0, min(80.0, self.cam_pitch - dy * 85.0))
        self.update_camera()
      self.mouse_dragging = True
    elif self.mouseWatcherNode.isButtonDown('mouse3'):
      if self.mouse_dragging:
        dy = my - self.last_mouse_y
        self.cam_dist = max(0.8, min(6.0, self.cam_dist - dy * 2.2))
        self.update_camera()
      self.mouse_dragging = True
    else:
      self.mouse_dragging = False

    self.last_mouse_x = mx
    self.last_mouse_y = my
    return Task.cont

  def update_motion(self, task):
    t = task.time
    theta_hip = 10.0 * math.sin(t * 2.0) + 3.0
    theta_knee = -26.0 * (math.sin(t * 2.0 - 0.4) + 1.0)
    theta_ankle = 7.0 * math.sin(t * 2.0)

    rh = math.radians(theta_hip)
    rk = math.radians(theta_knee)
    ra = math.radians(theta_ankle)

    self.hip_joint.setQuat(
        LQuaternion(math.cos(rh / 2.0), math.sin(rh / 2.0), 0, 0)
    )
    self.knee_joint.setQuat(
        LQuaternion(math.cos(rk / 2.0), math.sin(rk / 2.0), 0, 0)
    )
    self.ankle_joint.setQuat(
        LQuaternion(math.cos(ra / 2.0), math.sin(ra / 2.0), 0, 0)
    )

    return Task.cont


if __name__ == '__main__':
  app = RefinedLegViewer()
  app.run()
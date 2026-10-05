# -*- coding: utf-8 -*-
"""撞球任务：RamBallTask（对准前视球心直冲，四条判据任一命中即算撞上）

★ 判据优先级（逐行平移自旧 mission，逻辑不许改）：
  ① IMU 加速度超阈值（默认关 AUV_USE_IMU=False，需标定后开）
  ② 球宽突增 > AUV_RAM_W_PX（主判据，贴脸 = 撞上）
  ③ 曾贴脸（> AUV_RAM_W_NEAR_PX）后目标消失 > AUV_RAM_LOST_S
  ④ 垂向扰动（**开了垂向伺服就必须禁用**：伺服自己在改深度，
     dz / v_z 分不清"撞上了"还是"自己在动"，会误判 —— 这条注释是踩坑记录的结论）

★ 武装延迟 AUV_RAM_ARM_S：避开进入瞬间自身的加速度/水花扰动，防止一进门就误判撞上。
"""
from ..base import Task                                    # 任务基类
from ..servo import servo_depth, servo_yaw, clamp_depth    # 平移过来的伺服律


class RamBallTask(Task):
    """撞球：视觉对准 + 直冲 + 四条撞击判据"""

    name = 'RAM_BALL'

    def enter(self, ctx, now):
        """★ 每轮复位（4 个门共用类会踩的坑：max_w 不复位会直接误判穿过）"""
        self.label = self.gp(ctx, 'label_key', 'ball')
        self.aim = self.gp(ctx, 'aim_key', None)
        self.cam = self.lit('cam', 'front')
        self.armed = False                                  # 撞击判据是否已武装
        self.max_w = 0.0                                    # 期间见过的最大球宽
        self.last_seen = now                                # 最近一次看到球的时刻
        self.d0 = ctx.dep.get('D') if ctx.depth_ok() else None  # 进入时的深度（垂向扰动判据用）
        ctx.log('[AUV] %s 撞球开始（武装延迟 %.1fs）'
                % (self.name, self.gpf(ctx, 'ARM_S', 0.8)))

    def tick(self, ctx, now, dt):
        obs = ctx.see(self.cam, self.label, now, aim=self.aim)

        # ---- 武装延迟
        if not self.armed and ctx.elapsed(now) >= self.gpf(ctx, 'ARM_S', 0.8):
            self.armed = True
            self.last_seen = now

        if obs is not None:
            self.last_seen = now
            self.max_w = max(self.max_w, float(obs.get('w') or 0.0))
            # ★ 球的高度未知：用前视画面把球拉到瞄准点高度 + 机头正对它
            ctx.depth_cm = servo_depth(ctx, obs, dt, ctx.depth_cm)
            ctx.yaw_deg = servo_yaw(ctx, obs, dt, ctx.yaw_deg)
            sway = ctx.thrust(ctx.gf('AUV_SWAY_SIGN', 1.0) * ctx.gf('AUV_KP_SWAY', 0.60)
                              * float(obs.get('ex') or 0.0),
                              ctx.gf('AUV_SWAY_MAX', 0.50))
            surge = self.gpf(ctx, 'SURGE', 0.40)
            note = '对准中 dx=%+.0f dy=%+.0f w=%.0f D*=%.0fcm' \
                   % (obs.get('dx') or 0.0, obs.get('dy') or 0.0,
                      obs.get('w') or 0.0, ctx.depth_cm)
        else:
            sway = 0.0                                      # 目标丢失：保持最后修正方向、减速再找
            surge = self.gpf(ctx, 'SURGE', 0.40) * 0.5
            note = '球丢失，减速再捕获'

        if self.armed and self._hit(ctx, now, obs):
            return self.done(ctx, now, '撞击判定命中')

        r = self.timeout(ctx, now, 'TIMEOUT_S', 20.0, '撞球')
        if r is not None:
            return r
        return ctx.out(surge=surge, sway=sway, note=note)

    # ---------------------------------------------------------------- 撞击判据
    def _hit(self, ctx, now, obs):
        """四条判据任一命中 → True（返回前自行 done，保持与旧版一致的顺序）"""
        # ① IMU（遥测通 + 阈值标定后才启用）
        if ctx.g('AUV_USE_IMU', False):
            acc = ctx.tel_acc()
            thr = ctx.gf('AUV_IMPACT_ACC', 0.0)
            if acc is not None and thr > 0 and acc > thr:
                if ctx.holding('imu', True, now, ctx.gf('AUV_IMPACT_WINDOW_S', 0.3)):
                    ctx.log('[AUV] 撞击(IMU) acc=%.2f' % acc)
                    return True
            else:
                ctx.clear_hold('imu')
        # ② 尺度突增 = 贴脸
        if self.max_w > ctx.gf('AUV_RAM_W_PX', 220.0):
            ctx.log('[AUV] 撞击(尺度) 球宽 %.0f > 阈值' % self.max_w)
            return True
        # ③ 贴脸后消失
        if self.max_w > ctx.gf('AUV_RAM_W_NEAR_PX', 150.0) \
                and (now - self.last_seen) > ctx.gf('AUV_RAM_LOST_S', 0.8):
            ctx.log('[AUV] 撞击(消失) 贴脸后目标消失')
            return True
        # ④ 垂向扰动（⚠ 开了垂向伺服就必须禁用，见文件头说明）
        if ctx.depth_ok() and self.d0 is not None and not ctx.g('AUV_VERT_SERVO', True):
            dz = abs(float(ctx.dep['D']) - float(self.d0))
            vz = abs(float(ctx.dep.get('v_z') or 0.0))
            if dz > ctx.gf('AUV_RAM_DZ', 0.08) or vz > ctx.gf('AUV_RAM_VZ', 0.15):
                ctx.log('[AUV] 撞击(垂向) dz=%.3f vz=%.3f' % (dz, vz))
                return True
        return False

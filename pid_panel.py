"""七项控制 PID：独立编辑、明确编号映射、逐次确认后发送。"""
import json
import math
from pathlib import Path

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                             QLabel, QDoubleSpinBox, QSpinBox, QPushButton,
                             QCheckBox, QMessageBox, QFileDialog, QComboBox)

PID_LOOPS = (("pitch", "俯仰"), ("yaw", "偏航"), ("roll", "横摇"),
             ("depth", "深度"), ("gate", "过门"), ("hit_ball", "撞球"),
             ("pick_ball", "捡球"))
STM32_LOOPS = {'pitch': 0, 'yaw': 1, 'roll': 2, 'depth': 3}


class ControlPidPanel(QWidget):
    send_requested = pyqtSignal(str, int, float, float, float)
    status_sig = pyqtSignal(str)
    refresh_ports_requested = pyqtSignal()
    serial_toggle_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rows = {}
        layout = QVBoxLayout(self)
        self.hint = QLabel("姿态/深度发给 STM32；过门航向参数发给 S100。点击当前项“下发”并确认后才发送。")
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        transport = QHBoxLayout()
        transport.addWidget(QLabel("PID 下发链路"))
        self.transport_combo = QComboBox()
        self.transport_combo.addItems(["S100 UDP（姿态/深度及任务参数）", "直连 STM32 串口"])
        transport.addWidget(self.transport_combo)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(170)
        transport.addWidget(self.port_combo)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_combo.setCurrentText("115200")
        transport.addWidget(self.baud_combo)
        self.refresh_btn = QPushButton("刷新 PID 串口")
        self.refresh_btn.clicked.connect(self.refresh_ports_requested.emit)
        transport.addWidget(self.refresh_btn)
        self.serial_btn = QPushButton("连接 PID 串口")
        self.serial_btn.clicked.connect(self.serial_toggle_requested.emit)
        transport.addWidget(self.serial_btn)
        transport.addStretch(1)
        layout.addLayout(transport)
        self.link_status = QLabel("姿态/深度由S100转发STM32；过门参数由S100处理并确认。")
        self.link_status.setWordWrap(True)
        layout.addWidget(self.link_status)
        self.mapping_confirm = QCheckBox("允许下发 PID（已核对当前链路、编号和参数）")
        self.mapping_confirm.setToolTip("编号未指定或重复时禁止发送。更改编号或加载文件后需要重新核对。")
        self.mapping_confirm.toggled.connect(self._refresh_buttons)
        layout.addWidget(self.mapping_confirm)
        mapping_note = QLabel("STM32 0～3固定为俯仰/偏航/横摇/深度；4～7为参考补偿环。过门、撞球、捡球不占用这些编号。")
        mapping_note.setWordWrap(True)
        mapping_note.setStyleSheet("color:#8e5a18; font-size:12px;")
        layout.addWidget(mapping_note)
        table = QGridLayout()
        table.setHorizontalSpacing(12)
        table.setVerticalSpacing(10)
        for column, title in enumerate(("控制项", "执行端 / 编号", "P", "I", "D", "操作", "下发状态")):
            label = QLabel(title)
            label.setStyleSheet("font-weight:bold;")
            table.addWidget(label, 0, column)
        for row, (key, title) in enumerate(PID_LOOPS, 1):
            table.addWidget(QLabel(title), row, 0)
            index = QSpinBox()
            if key in STM32_LOOPS:
                number = STM32_LOOPS[key]
                index.setRange(number, number)
                index.setValue(number)
                index.setToolTip("STM32固定控制环编号")
            else:
                index.setRange(-1, -1)
                index.setSpecialValueText("S100" if key == 'gate' else "未接入")
                index.setToolTip("过门使用S100命名参数；撞球和捡球算法尚未接入")
            index.setEnabled(False)
            index.valueChanged.connect(self._mapping_changed)
            table.addWidget(index, row, 1)
            gains = []
            for column in range(2, 5):
                spin = QDoubleSpinBox()
                spin.setRange(-327.68, 327.67)
                spin.setDecimals(2)
                spin.setSingleStep(0.1)
                spin.setKeyboardTracking(False)
                spin.setMinimumWidth(130)
                spin.valueChanged.connect(lambda _value, loop=key: self._mark_modified(loop))
                table.addWidget(spin, row, column)
                gains.append(spin)
            send = QPushButton("下发 " + title)
            send.clicked.connect(lambda _checked=False, loop=key: self._confirm_send(loop))
            table.addWidget(send, row, 5)
            state = QLabel("未下发" if key in STM32_LOOPS or key == 'gate' else "算法未接入 · 禁止下发")
            state.setMinimumWidth(200)
            table.addWidget(state, row, 6)
            self._rows[key] = {"title": title, "index": index, "gains": gains,
                               "send": send, "state": state, "pending": False,
                               "target": 'stm32' if key in STM32_LOOPS else 's100',
                               "available": key in STM32_LOOPS or key == 'gate'}
            if key == 'gate':
                for spin, value in zip(gains, (30.0, 0.0, 12.0)):
                    spin.setValue(value)
                for spin in gains:
                    spin.setToolTip("P=过门航向比例，I=积分，D=角速度阻尼；横移比例保持S100当前配置")
        layout.addLayout(table)
        footer = QHBoxLayout()
        save = QPushButton("保存本地参数")
        save.clicked.connect(self._save)
        load = QPushButton("加载本地参数")
        load.clicked.connect(self._load)
        footer.addWidget(save)
        footer.addWidget(load)
        footer.addStretch(1)
        layout.addLayout(footer)
        self.delivery_note = QLabel("STM32参数提交后尚未回读确认。过门使用有线UDP，可收到S100参数更新确认；"
                                    "更新参数不会启动任务；过门任务按板端任务表运行。撞球/捡球可编辑保存，暂不可下发。")
        self.delivery_note.setWordWrap(True)
        layout.addWidget(self.delivery_note)
        layout.addStretch(1)
        self._refresh_buttons()
        self.transport_combo.currentIndexChanged.connect(self._transport_changed)
        self._transport_changed()

    def _transport_changed(self, *_args):
        direct = self.transport_combo.currentIndex() == 1
        for widget in (self.port_combo, self.baud_combo, self.refresh_btn, self.serial_btn):
            widget.setVisible(direct)
        self.mapping_confirm.setChecked(False)
        if not direct:
            self.link_status.setText("姿态/深度由S100转发STM32；过门参数由S100处理并确认。")
        self._refresh_buttons()

    def _mapping_changed(self, _value):
        self.mapping_confirm.setChecked(False)
        for row in self._rows.values():
            row["state"].setText("编号变更 · 请核对")
        self._refresh_buttons()

    def _mark_modified(self, key):
        if key in self._rows:
            row = self._rows[key]
            row["state"].setText("已修改 · 未下发" if row["available"] else "已编辑 · 算法未接入")

    def _refresh_buttons(self):
        for row in self._rows.values():
            allowed = (self.mapping_confirm.isChecked() and row['available'] and not row['pending']
                       and (row['target'] == 'stm32' or self.transport_combo.currentIndex() == 0))
            row["send"].setEnabled(allowed)
            row["send"].setToolTip("下发前将显示确认窗口" if allowed else "请填写不重复的板端编号，并核对对应关系")

    def _confirm_send(self, key):
        row = self._rows[key]
        if not row["send"].isEnabled():
            self.status_sig.emit("[PID] 未发送：请先核对板端编号")
            return
        for spin in row["gains"]:
            spin.interpretText()
        gains = tuple(spin.value() for spin in row["gains"])
        if not all(math.isfinite(value) for value in gains):
            self.status_sig.emit("[PID] 未发送：参数必须为有限数值")
            return
        index = row["index"].value()
        target = f"STM32 编号 {index}" if row['target'] == 'stm32' else "S100 过门航向对准参数"
        detail = (f"确认下发【{row['title']}】PID？\n\n"
                  f"执行端：{target}\nP = {gains[0]:.2f}\nI = {gains[1]:.2f}\nD = {gains[2]:.2f}\n"
                  f"下发链路：{self.transport_combo.currentText()}\n\n"
                  "该操作会提交给当前连接的机器。请确认编号和参数。")
        answer = QMessageBox.question(self, "确认下发 PID", detail,
                                      QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer != QMessageBox.Yes:
            row["state"].setText("已取消 · 未下发")
            return
        # 确认的是当前项的快照；编辑下一项不会触发任何自动下发。
        self.send_requested.emit(key, index, *gains)

    def mark_result(self, key, submitted, message=""):
        row = self._rows.get(key)
        if row is None:
            return
        row["state"].setText("已提交 · 未获板端确认" if submitted else (message or "发送失败"))
        row["state"].setStyleSheet("color:#1565c0;" if submitted else "color:#b71c1c;")

    def mark_task_pending(self, key):
        row = self._rows[key]
        row['pending'] = True
        row['state'].setText('已提交 · 等待S100确认')
        self._refresh_buttons()

    def mark_task_result(self, key, message, success=False):
        row = self._rows[key]
        row['pending'] = False
        row['state'].setText(message)
        row['state'].setStyleSheet('color:#1565c0;' if success else 'color:#b71c1c;')
        self._refresh_buttons()

    def settings(self):
        return {key: {"index": row["index"].value(),
                      "target": row['target'],
                      "p": row["gains"][0].value(), "i": row["gains"][1].value(),
                      "d": row["gains"][2].value()} for key, row in self._rows.items()}

    def apply_settings(self, data):
        # 先完整校验，避免加载错误文件导致半份参数被更新。
        if not isinstance(data, dict) or set(data) != set(self._rows):
            raise ValueError("参数文件必须包含指定的七项控制 PID，不能加载旧推进器参数文件")
        checked = {}
        for key, values in data.items():
            index = values["index"]
            gains = [float(values[name]) for name in ("p", "i", "d")]
            expected = STM32_LOOPS.get(key, -1)
            if type(index) is not int or index != expected or values.get('target') != self._rows[key]['target']:
                raise ValueError("执行端或编号不匹配；旧版七项编号文件不能加载")
            if not all(math.isfinite(value) and -327.68 <= value <= 327.67 for value in gains):
                raise ValueError("PID 参数超出允许范围")
            checked[key] = (index, gains)
        for key, (index, gains) in checked.items():
            row = self._rows[key]
            row["index"].setValue(index)
            for spin, value in zip(row["gains"], gains):
                spin.setValue(value)
            row["state"].setText("已加载 · 未下发" if row['available'] else "算法未接入 · 禁止下发")
        self.mapping_confirm.setChecked(False)
        self._refresh_buttons()

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存七项 PID", "control_pid_params.json", "JSON (*.json)")
        if not path:
            return
        try:
            Path(path).write_text(json.dumps({"schema": "rov-control-pid-v2", "loops": self.settings()},
                                             ensure_ascii=False, indent=2), encoding="utf-8")
            self.status_sig.emit("[PID] 本地参数已保存：" + path)
        except Exception as exc:
            self.status_sig.emit("[PID] 参数保存失败：" + str(exc))

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(self, "加载七项 PID", "", "JSON (*.json)")
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
            if data.get("schema") != "rov-control-pid-v2":
                raise ValueError("参数文件类型不匹配")
            self.apply_settings(data["loops"])
            self.status_sig.emit("[PID] 已加载本地参数，请核对编号后下发")
        except Exception as exc:
            self.status_sig.emit("[PID] 参数加载失败：" + str(exc))

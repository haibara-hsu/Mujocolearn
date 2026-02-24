# MuJoCo 6-Axis Robotic Arm Simulation Project

本專案包含一系列基於 **MuJoCo** 物理引擎的 Python 模擬腳本，涵蓋了從基礎 XML 建模到複雜的逆運動學 (IK) 與路徑規劃任務。

---

## 運行環境需求 — Requirement

在執行腳本前，請確保系統環境符合以下條件：

* **Python 版本**: 建議使用 Python 3.8+。
* **物理引擎**: `mujoco` (用於物理計算與渲染)。
* **數學運算**: `numpy` (逆運動學矩陣運算必備)。
* **互動界面**: 支援 `mujoco.viewer` 的圖形化顯示環境。

---

## 環境檔 (.env) 設定 — .env Setting

目前專案邏輯直接撰寫於 Python 腳本與 XML 字串中：
* **自填欄位**: 本專案目前不需要外部 `.env` 檔案。
* **硬體調整**: 若模擬運行緩慢，可在 XML 區塊中調整 `<option timestep="0.005"/>` 的數值。

---

## 本地端安裝與運行 — Build Setup (Local)

1. **安裝依賴套件**:
```bash
pip install mujoco numpy

```


2. **執行模擬腳本**:
* **基礎練習**: `python xml_practice.py` (觀察雙節棍物理特性)。
* **座標監控**: `python are_test.py` (即時印出末端 site 座標)。
* **數據紀錄**: `python learn_6axis.py` (將運動軌跡存為 `robot_data.csv`)。
* **任務演練**: `python trajectory_planning.py` (執行自動取放任務)。

3. **執行模擬腳本 (核心建議)**:

Windows / Linux:
python trajectory_planning.py

macOS (若上述指令無效):
mpython trajectory_planning.py
(註：mpython 能解決 macOS 視窗無法正常彈出或當機的問題)


---

## 檔案功能清單

| 檔名 | 技術重點 | 預期結果 |
| --- | --- | --- |
| `xml_practice.py` | XML 結構、Joint (鉸鏈) 設定 | 雙節棍受重力自然擺動 |
| `are_test.py` | 正運動學 (FK)、Site 追蹤 | 終端機顯示末端 X, Y, Z 座標 |
| `ik_tracking.py` | 逆運動學 (IK)、Jacobian 矩陣 | 手臂末端跟隨紅色目標球移動 |
| `learn_6axis.py` | CSV 輸出、Actuator 控制 | 產生 `robot_data.csv` 歷史紀錄 |
| `trajectory_planning.py` | 狀態機、Mocap 物件控制 | 自動完成「抓取、抬起、卸貨」流程 |

---

**如果你需要我針對其中某個檔案（例如 `trajectory_planning.py` 的 Pick & Place 邏輯）進行更詳細的流程圖說明，請告訴我！**

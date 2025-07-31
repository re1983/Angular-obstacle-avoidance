# Debug and Test Files

這個資料夾包含了在開發和調試 CBDR (Constant Bearing, Decreasing Range) 船隻避障系統過程中創建的所有測試和調試腳本。

## 文件說明

### 分析和調試類
- **`analyze_bearing_change.py`** - 分析bearing變化的詳細腳本，幫助理解bearing計算邏輯
- **`correct_analysis.py`** - 驗證修正後的bearing計算是否正確的分析腳本
- **`detailed_analysis.py`** - 詳細分析CBDR情況下的bearing變化和避障策略
- **`debug_avoidance.py`** - 調試避障邏輯的專用腳本
- **`debug_geometry.py`** - 調試幾何計算（bearing、距離等）的腳本
- **`debug_turning.py`** - 調試轉向決策邏輯，發現 np.sign(0.0) 問題的關鍵腳本
- **`rethink_logic.py`** - 重新思考和驗證避障邏輯的腳本

### 測試驗證類
- **`test_bearing_logic.py`** - 測試bearing計算邏輯的正確性
- **`test_corrected_logic.py`** - 測試修正後的避障邏輯
- **`test_correction.py`** - 測試最終修正版本的CBDR避障功能
- **`quick_verification.py`** - 快速驗證修正後避障行為的腳本

## 主要發現和修正

### 關鍵問題
1. **相對bearing vs 絕對bearing混淆** - 初始版本在CBDR檢測中使用了錯誤的bearing類型
2. **np.sign(0.0) 問題** - 當bearing變化為0時，船隻不會轉向
3. **轉向方向錯誤** - 船隻應該左轉避障，但實際右轉

### 解決方案
1. **使用絕對bearing進行CBDR檢測** - 確保正確識別恆定bearing情況
2. **特殊處理CBDR情況** - 當bearing rate ≈ 0時，根據相對位置決定轉向方向
3. **正確的避障邏輯** - Ship A在左舷時左轉，在右舷時右轉

## 使用方法

所有腳本都可以在主目錄中執行：
```bash
cd /home/re1983/extra/Work/Angular-obstacle-avoidance
python3 debug_tests/[script_name].py
```

或在 debug_tests 目錄中執行（需要調整 import 路徑）：
```bash
cd /home/re1983/extra/Work/Angular-obstacle-avoidance/debug_tests
python3 [script_name].py
```

## 開發時間線

1. **初始問題發現** - `debug_geometry.py`, `debug_avoidance.py`
2. **Bearing邏輯分析** - `analyze_bearing_change.py`, `test_bearing_logic.py`
3. **邏輯重構** - `rethink_logic.py`, `correct_analysis.py`
4. **轉向問題調試** - `debug_turning.py` (發現關鍵的 np.sign(0.0) 問題)
5. **詳細分析** - `detailed_analysis.py`
6. **最終測試** - `test_correction.py`, `quick_verification.py`

這些檔案記錄了完整的問題解決過程，可供將來參考或學習使用。

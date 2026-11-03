# 智慧鐵捲門控制器 ( NUCLEO-G474RE 期中專題 )

從傳統生硬控制走向人機互動美學，結合硬體 PWM 呼吸燈與 PLC 聯網日誌架構的新一代微型智慧鐵捲門控制系統。

## 1. 情境故事：誰、在哪、遇到什麼麻煩
我在趕著出門上課的時候，明明有關鐵捲門，但是到學校就會有個疑問：「我真的有關好嗎？」導致我上課心會懸在那邊。所以我製作一個智慧鐵捲門控制系統，可以透過遠端控制讓我可以隨時掌握鐵捲門的狀態，這樣就可以安心的上課了。

## 2. 替身對照：故事裡的模組 → 板上替身
- 真實鐵捲門上升 → PA5 綠燈（TIM2 PWM 漸亮）
- 真實鐵捲門下降 → PA6 黃燈（TIM3 PWM 漸亮）
- 鐵捲門到達/暫停狀態 → PA7 紅燈（GPIO 輸出）
- 緊急停止警報 → PB1 緊急 LED 與蜂鳴器
- 實體控制面板（上升/下降/停止/緊急）按鈕 → PC6 / PC7 / PC8 / PC9
- 雲端 / 手機遠端控制與監控 → USART2 終端機（Python 序列埠控制）
- 鐵捲門實際升降時間 → 時間縮放：真實數十秒 → 實測 5 秒

## 3. 功能與驗收條件：動作、預期、實測結果
1. **按下 PC6 (Btn_Up) 或輸入 `u`**：PA5 綠燈 5 秒內線性漸亮，UART 印出狀態，5 秒後自動停並亮紅燈 (PA7) 3 秒。
   - *實測結果*：PA5 綠燈從 0% 漸亮至 100%，5 秒後切換為紅燈亮 3 秒，UART 正常紀錄 ✅
2. **按下 PC7 (Btn_Down) 或輸入 `d`**：PA6 黃燈 5 秒內線性漸亮；UART 印出狀態，5 秒後自動停並亮紅燈 (PA7) 3 秒。
   - *實測結果*：PA6 黃燈從 0% 漸亮至 100%，5 秒後切換為紅燈亮 3 秒，UART 正常紀錄 ✅
3. **按下 PC8 (Btn_Stop) 或輸入 `s`**：切換至暫停狀態，PA7 紅燈點亮維持 3 秒後自動熄滅；UART 印出狀態。
   - *實測結果*：PA7 紅燈亮起，3 秒後自動關閉歸零，UART 正確紀錄 ✅
4. **按下 PC9 (Btn_Emergency)**：外部中斷觸發 EXTI 最高優先級中斷，PB1 緊急燈每 250ms 閃爍，UART 印出 EMERGENCY ALERT。
   - *實測結果*：瞬時進入急停，PB1 LED 規律閃爍，再次按下或輸入 `e` 可成功解鎖 ✅
5. **終端機輸入 `e` (遠端急停)**：立即觸發/解除緊急互鎖狀態，統計計數器 +1，UART 輸出操作次數。
   - *實測結果*：遠端指令無延遲切換急停狀態，UART 統計正確累計 ✅

## 4. 用到的技術：第 3 ~ 7 週勾選表 (同投影片)
- **時脈**：PLL 170 MHz (系統核心主頻與所有 Timer / PWM 計時基準)
- **GPIO**：
  - 輸出（停止告示/急停燈：PA7 停機亮 3 秒，PB1 急停閃爍；MODER = 01）
  - 輸入（實體按鈕：PC6(Up)、PC7(Down)、PC8(Stop) 輸入；MODER = 00，PUPDR = 01 內部上拉）
  - 複用功能（AF PWM 與 UART：PA5/PA6 驅動燈號，PA2/PA3 序列埠；MODER = 10，AFRL 選 AF1 與 AF7）
- **中斷**：
  - 外部中斷 EXTI（PC9 急停按鈕：下降沿觸發毫秒級切斷馬達；EXTICR3 選 PC9）
  - NVIC 致能（中斷服務處置：EXTI9_5 與 USART2 最高優先級設定）
- **Timer / PWM**：
  - 定時中斷（非阻塞時間管理：5 秒行程與 Uptime 計時，SysTick 1ms）
  - PWM 輸出（漸亮呼吸燈：PA5(綠/升) 與 PA6(黃/降) 0%~99% 漸亮；TIM2/TIM3）
- **UART**：
  - UART 傳送（PLC 日誌：輸出 Uptime 與 Up/Down/Stop/Emergency 次數；USART2 115200 8N1）
  - UART 接收 + 環形緩衝（遠端命令：解析 'u', 'd', 's', 'e' 指令控制）

## 5. 腳位表：用到的每支腳與用途
- `PA2` / `PA3` = USART2 (ST-LINK VCP 序列埠通訊)
- `PA5` = TIM2_CH1 (綠燈 5 秒 PWM 漸亮，模擬上升)
- `PA6` = TIM3_CH1 (黃燈 5 秒 PWM 漸亮，模擬下降)
- `PA7` = GPIO Output (紅燈，到達/暫停狀態告示 3 秒)
- `PB1` = GPIO Output (緊急 LED 與蜂鳴器閃爍)
- `PC6` = GPIO Input (實體上升按鈕，內部上拉)
- `PC7` = GPIO Input (實體下降按鈕，內部上拉)
- `PC8` = GPIO Input (實體停止按鈕，內部上拉)
- `PC9` = GPIO Input / EXTI9 (實體緊急按鈕，外部中斷觸發)

## 6. 建置與執行：開發環境、燒錄步驟、終端機設定、命令
- **開發環境**：STM32CubeIDE、HAL 庫
- **燒錄步驟**：透過 ST-LINK 將專案編譯並燒錄至 NUCLEO-G474RE 開發板。
- **終端機設定**：執行 `python shutter_test.py`，序列埠設定為 `115200 8N1`。
- **命令控制**：
  - `u`：啟動上升（綠燈呼吸漸亮）
  - `d`：啟動下降（黃燈呼吸漸亮）
  - `s`：停止 / 暫停（紅燈亮 3 秒）
  - `e`：切換 / 解除緊急互鎖狀態

## 7. 程式結構：哪個檔案做什麼、ISR 與主迴圈分工
- `main.c`：系統初始化（RCC、GPIO、TIM、USART、EXTI）、FSM 狀態機邏輯與主迴圈。
- `stm32g4xx_it.c`：中斷服務常式（ISR），包含 `EXTI9_5_IRQHandler`（處理緊急按鈕硬體中斷）與 `USART2_IRQHandler`（處理序列埠接收與環形緩衝區）。
- `tim.c` / `usart.c` / `gpio.c`：周邊硬體初始化配置檔。

## 8. 實拍照：每張附一行圖說
<img width="1145" height="1430" alt="接線" src="https://github.com/user-attachments/assets/44acf8cb-0853-4179-8285-fd99215edf9d" />
實作照片-接線圖
<img width="1673" height="978" alt="綠燈微亮1" src="https://github.com/user-attachments/assets/b61b2997-fcba-4a35-9b59-5b5d697ef803" />
<img width="1774" height="922" alt="綠燈全亮1" src="https://github.com/user-attachments/assets/5ae0b1c0-50e5-4318-a4c1-9e6be176feb2" />
實作照片-鐵捲門上升中
<img width="1691" height="968" alt="黃燈微亮1" src="https://github.com/user-attachments/assets/51f0e4bf-8cee-432f-82a9-2d46b9c2586a" />
<img width="1718" height="952" alt="黃燈全亮1" src="https://github.com/user-attachments/assets/bfeb9710-8013-4522-808f-58413a73dbeb" />
實作照片-鐵捲門下降中
<img width="1719" height="953" alt="停止紅燈1" src="https://github.com/user-attachments/assets/6924869f-7cbd-456c-9075-2b3778d6674f" />
實作照片-鐵捲門暫停
<img width="1705" height="960" alt="緊急紅燈1" src="https://github.com/user-attachments/assets/0cf11ffe-29d7-449f-a76a-3697efb45cc3" />
實作照片-鐵捲門緊急停止
<img width="922" height="1010" alt="終端機1" src="https://github.com/user-attachments/assets/b6352038-b5f2-423a-a183-d7352ea2613f" />
<img width="764" height="1090" alt="終端機2" src="https://github.com/user-attachments/assets/7327366b-f1bb-4ca2-94de-a12c5b099ffb" />

  實作照片-終端機狀態

## 9. AI 使用說明：哪段 AI、你怎麼查核
- **應用段落**：協助規劃狀態機（FSM）的邏輯架構、UART 環形緩衝區的接收實作，以及 Python 測試腳本的編寫。
- **查核方式**：透過示波器與實體 LED 驗證 PWM 呼吸燈週期是否精確為 5 秒；透過多次發送非同步指令與緊急中斷（EXTI），驗證系統的即時反應能力與 PLC 日誌數據累計是否正確無誤。

## 10. 已知限制：哪些用替身、換真貨要改哪裡
- **已知限制**：目前以 LED 呼吸燈與蜂鳴器模擬真實馬達的轉動與警報；實際應用於真實大型鐵捲門時，需將 GPIO 輸出訊號改接外部繼電器（Relay）與馬達驅動模組，並調整 PWM 週期與限位開關（Limit Switch）回授接腳。

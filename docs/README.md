# docs/

放演示素材。**推仓库之前先录一段**，README 顶部的图片引用才生效。

## 需要放的

| 文件名 | 内容 | 怎么录 |
|---|---|---|
| `demo.gif` | 倒立摆从倾斜到立正的动画 | 见下 |

## 怎么录

GUI 跑在你自己 Windows 桌面上（X11 转发），直接用录屏工具框住 matplotlib 窗口。

推荐 **[ScreenToGif](https://www.screentogif.com/)**（免费、便携）：

1. 先 `ros2 launch pendulum_control pendulum.launch.py` 把窗口跑起来
2. ScreenToGif → Recorder → 框住 matplotlib 窗口
3. 录 5-8 秒（够看到摆杆从倾斜 3° 晃几下立正、小车冲出去再回到原点）
4. 存成 GIF，**控制在 5 MB 以内**（GitHub 对大图不友好）
5. 命名 `demo.gif` 放进这个目录

## 录完要做的

回到 README.md，把顶部 HTML 注释里那行图片引用**解除注释**：

```markdown
![演示](docs/demo.gif)
```

（删掉行首的 `!- - ` 即可。）

## 建议再补一张

`result.png` —— 一张 x / θ 随时间变化的曲线图。
在 `visualizer.py` 里把轨迹存下来用 matplotlib 画一次就有了。

> 面试场景：对方问"效果怎么样"，图比嘴快。

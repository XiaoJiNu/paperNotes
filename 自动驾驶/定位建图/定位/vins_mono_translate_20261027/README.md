# VINS-Mono 中文全文翻译

## 源论文
- **标题：** VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator
- **作者：** Tong Qin, Peiliang Li, Shaojie Shen
- **来源 PDF：** `2017-VINS-Mono.pdf`（arXiv:1708.03852v1 [cs.RO], 13 Aug 2017）
- **页数：** 17

## 交付物
| 文件 | 说明 |
|------|------|
| `VINS-Mono_中文全文翻译.md` | 完整中文 Markdown 翻译（含全部章节、公式、图表、参考文献） |
| `VINS-Mono_核心方法与代码讲解.md` | 核心方法步骤 + 论文对照 + 开源仓库代码落点讲解 |
| `VINS-Mono_中文全文翻译-20261103.md` | 2026-11-03 深度导读与公式精解（全插图） |
| `figs/` | 从图中提取/裁切的图片资源（fig01–fig25） |
| `2017-VINS-Mono.pdf` | 原始 PDF |

## 统计
- **页数：** 17
- **图（Figure）：** 25（Fig. 1–25，均已嵌入 Markdown）
- **表（Table）：** 1（Table I 时间统计，Markdown 表格）
- **算法框：** Algorithm 1（寻找 b1 与 b2）
- **公式编号：** (1)–(29)，以 LaTeX `$$...$$` + `\tag{N}` 保留
- **翻译状态：** **完整**（标题、作者、摘要、第 I–X 节、致谢性质内容、全部参考文献均已中文对应；未做摘要式省略）

## 图清单自检
Fig.1–Fig.25 均存在于 `figs/figNN.png`（或组合面板）并在 md 中以 `![中文图题](figs/...)` 引用。

## 表清单自检
Table I Timing Statistics → 中文「表 I　时间统计」，数值保持原样。

## 说明
- 示意图类图片多数由 PDF 页面渲染后裁切；照片类优先使用 `pdfimages` 抽取。
- 部分裁切图可能仍带英文坐标轴/图例文字（保留原图信息）；图题已全部中文化。
- 相对路径图片链接：`figs/fig01.png` 等，在同目录打开 Markdown 即可渲染。

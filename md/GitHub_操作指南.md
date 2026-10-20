# GitHub 操作指南（本工程 WPT 仓库）

> 面向工程交接：说明如何拉取、修改、上传本工程到 GitHub，以及常见问题的排查。
> 本工程远程仓库：`https://github.com/hahuihei/WPT`，默认分支 `main`。

---

## 1. 仓库信息

| 项 | 值 |
|----|-----|
| 远程地址 | `https://github.com/hahuihei/WPT` |
| 本地目录 | `D:\wenjian\OneDrive\workspace_v6_1_3\DSP8233x_BUCKSSTC` |
| 默认分支 | `main` |

查看当前远程与分支：

```powershell
git remote -v      # 查看远程地址
git branch -vv     # 查看当前分支及跟踪关系
git status         # 查看工作区改动状态
```

---

## 2. 环境准备（一次性）

### 2.1 安装 Git

从 https://git-scm.com/download/win 下载安装，安装后用 Git Bash 或 PowerShell 验证：

```powershell
git --version
```

### 2.2 配置用户名和邮箱（提交时显示）

```powershell
git config --global user.name  "你的名字"
git config --global user.email "你的邮箱"
```

### 2.3 配置代理（国内网络访问 GitHub 通常需要）

本工程当前已配置代理到本地 `127.0.0.1:7890`（Clash 等代理软件默认端口）：

```powershell
# 设置代理
git config --global http.proxy  http://127.0.0.1:7890
git config --global https.proxy http://127.0.0.1:7890

# 取消代理（能直连 GitHub 时）
git config --global --unset http.proxy
git config --global --unset https.proxy

# 查看当前代理配置
git config --get http.proxy
git config --get https.proxy
```

### 2.4 认证方式（二选一）

GitHub 已不支持密码登录 HTTPS 推送，需用以下任一种：

**方式 A — HTTPS + 个人访问令牌（PAT）**

1. GitHub → Settings → Developer settings → Personal access tokens → Generate new token，勾选 `repo` 权限。
2. 首次 `git push` 时用户名填 GitHub 账号，密码填 **token**（不是登录密码）。

**方式 B — SSH（推荐，免密且更稳）**

```powershell
# 生成密钥（一路回车）
ssh-keygen -t ed25519 -C "你的邮箱"

# 查看公钥内容
cat ~/.ssh/id_ed25519.pub
```

把公钥内容添加到 GitHub → Settings → SSH and GPG keys → New SSH key，然后切换远程为 SSH：

```powershell
git remote set-url origin git@github.com:hahuihei/WPT.git
```

---

## 3. 拉取 / 读取代码

### 3.1 首次克隆（新机器）

```powershell
git clone https://github.com/hahuihei/WPT.git
# 或 SSH：
git clone git@github.com:hahuihei/WPT.git
```

### 3.2 拉取最新更新

```powershell
git pull          # 拉取并合并远程更新（推荐日常用）
git fetch         # 只拉取，不合并
```

> 若本地有未提交改动，先提交或暂存（`git stash`）再 pull，避免冲突。

### 3.3 查看历史与改动

```powershell
git log --oneline -10     # 最近 10 条提交
git log --stat            # 查看每次提交改了哪些文件
git diff                  # 查看未暂存的改动
git diff --stat           # 改动统计
git show <commit-id>      # 查看某次提交详情
```

---

## 4. 上传代码（提交 + 推送）

标准三步：**add → commit → push**。

```powershell
# 1. 暂存改动（把文件加入提交列表）
git add <文件名>          # 指定文件
git add -A                # 暂存所有改动（含新增/删除）

# 2. 提交（生成一次快照）
git commit -m "提交说明"

# 3. 推送到远程
git push
```

### 4.1 提交说明建议

一行中文，说明「做了什么 + 为什么」：

```
git commit -m "启用 SSTC 软约束与 MPC 控制，修正 BUCK_UF 为 25V，重写工程文档"
```

### 4.2 完整示例

```powershell
cd D:\wenjian\OneDrive\workspace_v6_1_3\DSP8233x_BUCKSSTC
git status                 # 先看改了什么
git add -A
git commit -m "xxx 功能修改"
git push
```

---

## 5. 分支操作

```powershell
git branch                # 查看本地分支
git branch <名字>         # 新建分支
git checkout <名字>       # 切换分支
git checkout -b <名字>    # 新建并切换

# 推送新分支到远程
git push -u origin <名字>

# 合并分支到 main
git checkout main
git merge <名字>
```

---

## 6. 撤销与回滚

```powershell
# 撤销对某文件的未暂存改动（危险，会丢改动）
git restore <文件名>

# 撤销最后一次 commit（保留改动在工作区）
git reset --soft HEAD~1

# 回退到某个历史版本（丢弃之后的提交，危险）
git reset --hard <commit-id>

# 丢弃工作区所有未提交改动（危险）
git checkout .
```

> 带 `--hard` / `checkout .` 的命令会不可逆地丢弃改动，操作前先确认。

---

## 7. 常见问题

### 7.1 push 报 `Failed to connect ... over proxy`

代理没开。启动代理软件（Clash 等），或按 2.3 节取消/重设代理。

### 7.2 push 报 `Recv failure: Connection was reset`

代理已开但连接被重置，通常是代理节点不稳或 GitHub 被干扰。尝试：

1. 代理软件里**切换节点**（换一个延迟低的节点）；
2. 改用 **SSH 方式**（见 2.4 节方式 B），SSH 走 22 端口通常更稳；
3. 关闭系统代理后按 2.3 节取消 Git 代理，直连重试。

### 7.3 报 `Authentication failed`

HTTPS 推送时密码应填 **Personal Access Token**，不是 GitHub 登录密码（见 2.4 节）。

### 7.4 pull 时报冲突（conflict）

```powershell
git pull
# 手动编辑冲突文件，解决 <<<<<<< / ======= / >>>>>>> 标记
git add <解决后的文件>
git commit -m "解决冲突"
git push
```

### 7.5 忘了加 `.gitignore`

若某些文件（如编译产物、`.ccsproject` 等）不该上传，编辑项目根目录 `.gitignore`，添加忽略规则后提交：

```powershell
git add .gitignore
git commit -m "更新 .gitignore"
git push
```

---

## 8. 速查命令表

| 操作 | 命令 |
|------|------|
| 查看状态 | `git status` |
| 查看改动 | `git diff` |
| 查看历史 | `git log --oneline` |
| 暂存 | `git add <文件>` / `git add -A` |
| 提交 | `git commit -m "说明"` |
| 推送 | `git push` |
| 拉取 | `git pull` |
| 查看远程 | `git remote -v` |
| 切换分支 | `git checkout <分支>` |

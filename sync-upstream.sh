#!/usr/bin/env bash
# sync-upstream.sh —— 把 OLED 版的反向移植同步到最新上游，并推送到 fork
#
# 做三件事（默认全做）:
#   1) fetch 上游 master（github 直连失败时自动走 gh-proxy 镜像，再不行用本地上游克隆）
#   2) 把"上游 + 1 个 port 提交"里的那个提交 rebase 到新上游上
#   3) 推送到 fork（force-with-lease，目标默认 fork 的 master）
#
# 用法:
#   ./sync-upstream.sh                  # 同步 + 推送到 fork
#   ./sync-upstream.sh --check          # 只查状态（含"上次有没有没做完的"），什么都不改
#   ./sync-upstream.sh --no-push        # 只 rebase，不推送
#   ./sync-upstream.sh --build          # rebase 后跑一次构建验证，再推送
#   ./sync-upstream.sh --repo DIR       # 指定要同步的仓库（默认 <脚本目录>/reverse-port）
#   ./sync-upstream.sh --fork URL       # 指定 fork（默认下面的 FORK_URL）
#   ./sync-upstream.sh --force          # 工作区不干净也继续（谨慎）
#
# 中断恢复:
#   脚本每次 rebase 前会打一个备份点 refs/backup/pre-sync-<时间戳>（只存本地，不会被推送），
#   回滚: git reset --hard <备份点>。再次运行脚本时会自动：
#     - 清掉上次中断留下的临时 remote
#     - 若 rebase 已解决但没 continue → 自动 continue；还有冲突 → 列出文件并停下
#   冲突解决: git add <文件> && git rebase --continue（或 git rebase --abort 放弃）
# 细节与原则见 REBASE_NOTES.md。

if [ -z "${BASH_VERSION:-}" ]; then
  echo "错误: 这是 bash 脚本，请用 ./sync-upstream.sh 或 bash sync-upstream.sh 运行" >&2
  exit 1
fi
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认仓库：优先 DS5_SRC；其次脚本旁边的 reverse-port/（VM 上的布局）；
# 否则就用脚本自己所在的目录（脚本放进仓库时的用法）
if [ -n "${DS5_SRC:-}" ]; then
  REPO="$DS5_SRC"
elif [ -d "$HERE/reverse-port/.git" ]; then
  REPO="$HERE/reverse-port"
else
  REPO="$HERE"
fi
UPSTREAM_URL="${UPSTREAM_URL:-https://github.com/awalol/DS5Dongle.git}"
MIRROR_PREFIX="${MIRROR_PREFIX:-https://gh-proxy.com/}"
FORK_URL="${FORK_URL:-https://github.com/CLF184/DS5Dongle.git}"
FORK_BRANCH="${FORK_BRANCH:-master}"
LOCAL_UPSTREAM="${LOCAL_UPSTREAM:-$HERE/deps/upstream-DS5Dongle}"   # 兜底：本地上游克隆

DO_PUSH=1; DO_BUILD=0; CHECK_ONLY=0; FORCE=0
while [ $# -gt 0 ]; do
  case "$1" in
    --no-push) DO_PUSH=0 ;;
    --build)   DO_BUILD=1 ;;
    --check)   CHECK_ONLY=1 ;;
    --force)   FORCE=1 ;;
    --repo)    REPO="$2"; shift ;;
    --fork)    FORK_URL="$2"; shift ;;
    --upstream) UPSTREAM_URL="$2"; shift ;;
    -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（--help 看用法）" >&2; exit 2 ;;
  esac
  shift
done

cd "$REPO"

# ============ 0) 启动自检：上次是否没做完 ============
REBASE_DIR="$(git rev-parse --git-path rebase-merge)"
REBASE_APPLY="$(git rev-parse --git-path rebase-apply)"
UNAPPLIED=$(git status --short | grep -cE "^(UU|AA|DU|UD|AU|UA|DD)" || true)

if [ -d "$REBASE_DIR" ] || [ -d "$REBASE_APPLY" ]; then
  echo "== 检测到上次的 rebase 还没做完"
  if [ "$UNAPPLIED" -gt 0 ]; then
    echo "!! 还有未解决的冲突文件："
    git status --short | grep -E "^(UU|AA|DU|UD|AU|UA|DD)" || true
    echo "   解决： git add <文件>  &&  git rebase --continue"
    echo "   放弃： git rebase --abort     （回到 rebase 前的状态）"
    exit 1
  fi
  if [ "$CHECK_ONLY" -eq 1 ]; then
    echo "   （冲突已解决、只差 continue；正式跑一次脚本会自动完成它）"
    exit 1
  fi
  echo "== 冲突已解决但没 continue，自动继续 ..."
  GIT_EDITOR=true git rebase --continue
  echo "== 已继续完成: $(git log --oneline -1)"
fi

# 上次中断留下的临时 remote / 备份点
if git remote | grep -qx _sync_fork; then
  echo "== 清理上次留下的临时 remote _sync_fork"
  [ "$CHECK_ONLY" -eq 1 ] || git remote remove _sync_fork
fi
if git for-each-ref refs/backup/ | grep -q .; then
  echo "== 已有备份点（回滚: git reset --hard <名字>；只存本地，不会被推送）:"
  git for-each-ref --sort=-creatordate --format='   %(refname:short)  ->  %(objectname:short)  %(contents:subject)' refs/backup/ | head -5
fi

if [ "$CHECK_ONLY" -eq 0 ] && [ -n "$(git status --porcelain)" ] && [ "$FORCE" -eq 0 ]; then
  echo "!! 工作区不干净，先提交/暂存（或用 --force 强制继续）：" >&2
  git status --short | head -20 >&2
  exit 1
fi

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
echo "== 仓库: $REPO（分支 $BRANCH）"
echo "== 当前: $(git log --oneline -1)"

# ---- 1) fetch 上游（直连 → 镜像 → 本地上游克隆）----
fetch_upstream() {
  local url="$1" label="$2"
  if git fetch -q "$url" master 2>/dev/null; then
    echo "== 已从 $label 获取上游"
    return 0
  fi
  return 1
}
echo "== 获取上游 master ..."
if   fetch_upstream "$UPSTREAM_URL" "github"; then :
elif fetch_upstream "${MIRROR_PREFIX}${UPSTREAM_URL}" "镜像站"; then :
elif [ -d "$LOCAL_UPSTREAM/.git" ] && fetch_upstream "$LOCAL_UPSTREAM" "本地上游克隆（可能不是最新！）"; then :
else
  echo "!! 三种方式都拿不到上游（github 直连 / 镜像站 / 本地克隆）" >&2
  exit 1
fi
UP_NEW="$(git rev-parse FETCH_HEAD)"
BASE="$(git merge-base HEAD "$UP_NEW")"
echo "== 上游最新: $(git log --oneline -1 "$UP_NEW")"
echo "== 我们的基点: $(git log --oneline -1 "$BASE")"
echo "== 本分支自己的提交数: $(git rev-list --count "$BASE"..HEAD)"

if [ "$UP_NEW" = "$BASE" ]; then
  echo "== 已经基于最新上游，无需 rebase"
else
  echo "== 上游新增 $(git rev-list --count "$BASE".."$UP_NEW") 个提交 -> rebase 我们的改动"
  if [ "$CHECK_ONLY" -eq 1 ]; then
    echo "== (--check 模式，不做任何修改)"
    exit 0
  fi
  BACKUP="refs/backup/pre-sync-$(date +%Y%m%d-%H%M%S)"
  git update-ref "$BACKUP" HEAD
  echo "== 已备份当前状态到 $BACKUP"
  if ! git rebase --onto "$UP_NEW" "$BASE"; then
    echo
    echo "!! rebase 冲突，需要手工解决。冲突文件："
    git status --short | grep -E "^(UU|AA|DU|UD|AU|UA|DD)" || git status --short | head -20
    echo
    echo "   解决后：  git add <文件>  &&  git rebase --continue"
    echo "   然后重跑本脚本（会自动跳过已完成的 rebase 直接推送）"
    echo "   放弃：    git rebase --abort   （或 git reset --hard $BACKUP 回到 rebase 前）"
    exit 1
  fi
  echo "== rebase 完成: $(git log --oneline -1)"
fi

# ---- 2) 可选构建验证 ----
if [ "$DO_BUILD" -eq 1 ]; then
  echo "== 构建验证 ..."
  DS5_SRC="$REPO" \
  DS5_SDK="${DS5_SDK:-/home/user/project/toolchain/pico-sdk-2.3.0}" \
  DS5_TOOLCHAIN="${DS5_TOOLCHAIN:-/home/user/project/toolchain/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi}" \
  "${BUILD_SH:-$HERE/build.sh}" -DPICOTOOL_FETCH_FROM_GIT_PATH="${PICOTOOL_DEPS:-$HERE/deps/sdk23-deps}"
  echo "== 构建通过"
fi

# ---- 3) 推送到 fork ----
if [ "$DO_PUSH" -eq 0 ] || [ "$CHECK_ONLY" -eq 1 ]; then
  echo "== (跳过推送) 本地已完成同步"
  exit 0
fi

echo "== 推送 $BRANCH -> $FORK_URL ($FORK_BRANCH) ..."
git remote remove _sync_fork 2>/dev/null || true
git remote add _sync_fork "$FORK_URL"
# force-with-lease 需要知道远端当前值：先 fetch（失败就不推，避免盲推覆盖）
if ! git fetch -q _sync_fork "$FORK_BRANCH" 2>/dev/null; then
  if ! git fetch -q "${MIRROR_PREFIX}${FORK_URL}" "$FORK_BRANCH" 2>/dev/null; then
    echo "!! 无法读取 fork 当前状态（直连/镜像都不通），为安全起见不推送。" >&2
    echo "   稍后网络恢复（或主机代理开着）再重跑本脚本即可。" >&2
    git remote remove _sync_fork 2>/dev/null || true
    exit 1
  fi
fi
if git push _sync_fork "HEAD:refs/heads/$FORK_BRANCH" --force-with-lease; then
  echo "== 推送完成: $(git log --oneline -1)"
else
  echo "!! 推送失败（凭据 / 网络 / 远端有意外更新）。本地已同步好，修复后可重跑。" >&2
  exit 1
fi
git remote remove _sync_fork 2>/dev/null || true

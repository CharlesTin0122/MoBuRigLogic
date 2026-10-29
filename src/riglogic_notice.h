#ifndef MOBURIGLOGIC_NOTICE_H
#define MOBURIGLOGIC_NOTICE_H
/**
 * DNA 解析通知队列（进程级，只在主线程使用）
 *
 * 为什么要单独抽一层：DNA 解析发生在场景加载（FbxRetrieve）与绑定重建
 * （SetupAllAnimationNodes）里，那里**绝不能弹模态框**——加载中弹框会嵌套事件循环，
 * MoBu 2019 很容易卡死；一次 merge 多个角色还会连弹很多个框。
 *
 * 所以解析时只“登记通知”，由主线程 UI 空闲统一排空成一条汇总消息：
 *   - 自动改写成功的节点 → 一条消息 + [立即保存] / [稍后]
 *   - 未找到可用 DNA 的节点 → 一条消息 + [知道了]
 * 同一节点同一组路径只提示一次（去重），避免反复 Rebuild 时骚扰。
 *
 * 静默方式（脚本/批量）：
 *   - 对话框勾选“本次不再提示”，本次会话内不再弹；
 *   - 或启动前设环境变量 MOBURIGLOGIC_NO_DNA_NOTICES=1。
 */

#include <fbsdk/fbsdk.h>

#include <cstdlib>
#include <set>
#include <string>
#include <vector>

namespace moburiglogic {

//! 一次 DNA 解析的结果，用于汇总提示
struct DnaNotice {
    bool                     relocated = false;  // true=已自动改写 DNA Path；false=未找到可用 DNA
    std::string              node;               // 约束名
    std::string              ns;                 // 角色 namespace（可为空）
    std::string              oldPath;            // 解析前的 DNA Path
    std::string              newPath;            // 实际使用的 DNA（relocated 时有效）
    std::vector<std::string> tried;              // 尝试过的候选（not found 时用于诊断）
};

class DnaNoticeQueue
{
public:
    //! 弹窗是否启用（环境变量 > 会话内“不再提示” > 默认开）
    static bool Enabled()
    {
        static const bool envOff = []() {
            const char* v = std::getenv( "MOBURIGLOGIC_NO_DNA_NOTICES" );
            return v && *v && *v != '0';
        }();
        return !envOff && sEnabled;
    }

    //! 供脚本关闭（会话级）
    static void SetEnabled( bool on ) { sEnabled = on; }

    /** 场景被打开/新建：清空去重表并重置节流。
        不动待发队列——加载回调与文件事件的先后顺序在 MoBu 里不保证。
        去重键里含约束指针，跨文件本来就不会误判。 */
    static void OnSceneLoaded()
    {
        sSeen.clear();
        sIdle = 0;
    }

    static void Push( const DnaNotice& notice )
    {
        if (!notice.relocated && notice.tried.empty()) return;   // 没试过任何路径：不打扰

        std::string key = notice.node;
        key += notice.relocated ? "|R|" : "|F|";
        key += notice.oldPath;
        key += "|";
        key += notice.relocated ? notice.newPath : Join( notice.tried );
        if (!sSeen.insert( key ).second) return;                  // 已提示过

        // 同一节点只保留最终结论：加载早期（场景路径还没就绪）会先推一条“未找到”，
        // 随后回退成功又推一条“已改写”——两条同时出现会让提示自相矛盾。
        // 规则：已改写优先；新的“未找到”在已有改写结果时直接丢弃
        for (auto it = sQueue.begin(); it != sQueue.end(); )
        {
            if (it->node != notice.node) { ++it; continue; }
            if (!notice.relocated && it->relocated) return;       // 已有改写结果，忽略失败
            it = sQueue.erase( it );
        }

        sQueue.push_back( notice );
        sIdle = 0;
    }

    /** 由 UI 空闲回调调用。
        \param playing 传输器是否在播放：播放中不弹，等停止后再说。 */
    static void DrainIfReady( bool playing )
    {
        if (sQueue.empty() || playing || !Enabled()) return;

        // 加载/重建刚结束时求值线程可能还在收尾，等空闲稳定后再弹
        if (++sIdle < kSettleTicks) return;
        sIdle = 0;

        std::vector<DnaNotice> batch;
        batch.swap( sQueue );

        std::string okBlock, failBlock;
        int okCount = 0, failCount = 0;
        for (const auto& n : batch)
        {
            std::string label = n.node;
            if (!n.ns.empty()) label += " [" + StripColon( n.ns ) + "]";
            if (n.relocated)
            {
                ++okCount;
                okBlock += "\n  - " + label + "\n";
                okBlock += "      旧：" + Show( n.oldPath ) + "\n";
                okBlock += "      新：" + n.newPath + "\n";
            }
            else
            {
                ++failCount;
                failBlock += "\n  - " + label + "\n";
                failBlock += "      DNA Path：" + Show( n.oldPath ) + "\n";
                failBlock += "      已尝试：";
                for (std::size_t i = 0; i < n.tried.size(); ++i)
                {
                    if (i) failBlock += "\n                ";
                    failBlock += std::to_string( i + 1 ) + ") " + n.tried[i];
                }
                failBlock += "\n";
            }
        }

        std::string msg;
        if (okCount)
        {
            msg += "DNA 路径已自动更新（" + std::to_string( okCount ) + " 个节点）\n";
            msg += okBlock;
            msg += "\n文件已被修改，请保存后继续使用。";
        }
        if (failCount)
        {
            if (okCount) msg += "\n\n";
            msg += "未找到可用的 DNA 文件（" + std::to_string( failCount ) + " 个节点）\n";
            msg += failBlock;
            msg += "\n请在约束面板的 DNA Path 中手动指定 DNA 文件，然后 Rebuild Bindings 并保存。";
        }

        // 用 FBMessageBox（2019 实测可用，返回 1 基按钮序号）；
        // 不用带复选框的 WithCheck 变体：实测在某些情况下弹不出来。
        // 末位按钮固定为“不再提示本次”，其余按位映射。
        const bool canSave = okCount > 0;
        const int pressed = canSave
            ? FBMessageBox( "RigLogic DNA", msg.c_str(),
                            "立即保存", "稍后", "不再提示本次", 0, /*pScrolledMessage=*/1 )
            : FBMessageBox( "RigLogic DNA", msg.c_str(),
                            "知道了", "不再提示本次", nullptr, 0, /*pScrolledMessage=*/1 );

        const int muteButton = canSave ? 3 : 2;
        if (pressed == muteButton) { sEnabled = false; return; }

        if (canSave && pressed == 1)
        {
            const char* cur = FBApplication().FBXFileName.AsString();
            FBApplication().FileSave( (cur && *cur) ? cur : nullptr );
        }
    }

private:
    static constexpr int kSettleTicks = 30;   // ≈0.5~1 秒空闲

    static std::string StripColon( const std::string& ns )
    {
        return (!ns.empty() && ns.back() == ':') ? ns.substr( 0, ns.size() - 1 ) : ns;
    }
    static std::string Show( const std::string& path )
    {
        return path.empty() ? std::string( "<空>" ) : path + "（文件不存在）";
    }
    static std::string Join( const std::vector<std::string>& items )
    {
        std::string out;
        for (const auto& s : items) { out += s; out += ';'; }
        return out;
    }

    inline static std::vector<DnaNotice> sQueue;
    inline static std::set<std::string>  sSeen;
    inline static int                    sIdle = 0;
    inline static bool                   sEnabled = true;
};

} // namespace moburiglogic

#endif // MOBURIGLOGIC_NOTICE_H

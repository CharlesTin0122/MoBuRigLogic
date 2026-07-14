/**
 * 插件库入口 —— MotionBuilder 2024 加载点
 */
#include <fbsdk/fbsdk.h>

FBLibraryDeclare( moburiglogic )
{
    FBLibraryRegister( RigLogicBodyConstraint );
    FBLibraryRegister( RigLogicHeadConstraint );
    FBLibraryRegister( RigLogicBodyLayout );
    FBLibraryRegister( RigLogicHeadLayout );
}
FBLibraryDeclareEnd;

bool FBLibrary::LibInit()    { return true; }
bool FBLibrary::LibOpen()    { return true; }
bool FBLibrary::LibReady()   { return true; }
bool FBLibrary::LibClose()   { return true; }
bool FBLibrary::LibRelease() { return true; }

#include "sounddefinitions.h"

#include <Tempest/Log>
#include <gothic.h>

using namespace Tempest;

SoundDefinitions::SoundDefinitions() {
  auto vm = Gothic::inst().createVm(u"Sfx.dat");

  vm->getDATFile().iterateSymbolsOfClass("C_SFX",[this,&vm](size_t i,Daedalus::PARSymbol& s){
    Daedalus::GEngineClasses::C_SFX sfx;
    vm->initializeInstance(sfx, i, Daedalus::IC_Sfx);
    vm->clearReferences(Daedalus::IC_Sfx);
    this->sfx[s.name] = std::move(sfx);
    });
  }

const Daedalus::GEngineClasses::C_SFX& SoundDefinitions::operator[](const char *name) const {
  auto i = sfx.find(name);
  if(i!=sfx.end())
    return i->second;
  static Daedalus::GEngineClasses::C_SFX s;
  return s;
  }


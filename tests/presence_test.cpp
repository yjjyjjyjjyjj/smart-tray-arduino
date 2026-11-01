#include "../main_edit.c/PresenceTracker.h"
#include <assert.h>
#include <stdio.h>

struct Rig {
  PresenceTracker p;
  uint32_t now;
  int32_t ref, scale;
  Rig(int32_t s=1000): now(0), ref(100000), scale(s) { p.reset(ref); }
  void sample(int dg) {
    now += 100;
    p.update(ref + dg*scale/100, now, ref, scale, 400, 250, 120);
  }
  void hold(int dg, int n=35) { for(int i=0;i<n;i++) sample(dg); }
};
int main() {
  {
    Rig r; r.hold(1500);
    uint32_t removedAt=r.now;
    r.hold(50,4); // 10 SPS: median + three consecutive release samples
    assert(!r.p.present && r.now-removedAt<=400);
    assert(r.p.zero==r.ref); // fast LED OFF must not instantly retare
    r.hold(50); assert(r.p.weight==0 && r.p.zero!=r.ref);
  }
  {
    PresenceTracker p; const int32_t ref=-572354, scale=-293;
    uint32_t now=0; p.reset(ref);
    for(int i=0;i<30;i++) p.update(-809375,now+=100,ref,scale,400,250,120);
    assert(p.present);
    // A single released-looking sample must not switch the LED off.
    p.update(-547689,now+=100,ref,scale,400,250,120);
    assert(p.present);
    for(int i=0;i<30;i++) p.update(-540556+(i%3-1)*100,now+=100,ref,scale,400,250,120);
    assert(!p.present && p.zero==ref);
    for(int i=0;i<30;i++) p.update(-809375,now+=100,ref,scale,400,250,120);
    assert(p.present);
  }
  {
    // Real T2 unloaded trace replay with its reported reference and signed scale.
    PresenceTracker p; const int32_t reference=-266395, scale=1657;
    p.reset(reference); uint32_t now=0;
    for(int i=0;i<35;i++) p.update(reference+49710,now+=100,reference,scale,1000,500,120);
    assert(p.present);
    const int32_t raw[]={-270192,-270087,-270159,-270122,-270199,-270033,
        -270081,-270123,-270293,-270184,-270171,-270035,-270210};
    for(int i=0;i<40;i++) p.update(raw[i%13],now+=100,reference,scale,1000,500,120);
    assert(!p.present);
    assert(PresenceTracker::magnitude((p.zero-reference)*100/scale)<=240);
  }
  assert(PresenceTracker::validRelease(1000,2000)==500);
  assert(PresenceTracker::validRelease(400,250)==250);
  assert(PresenceTracker::validRelease(400,0)==200);
  assert(PresenceTracker::calibrationTolerance(250,false)==512);
  assert(PresenceTracker::calibrationTolerance(-1000,true)==100);
  {
    const int32_t readings[]={-505059,-505044,-505075,-504978,-504978,-505049,
        -504905,-504821,-505073,-504915,-504824,-504945};
    StableSamples oldWindow={}, bootstrap={};
    bool oldAccepted=false, newAccepted=false;
    for (uint32_t i=0;i<36;i++) {
      oldAccepted |= oldWindow.add(readings[i%12],i*100,25);
      newAccepted |= bootstrap.add(readings[i%12],i*100,512);
    }
    assert(!oldAccepted && newAccepted);
    // Synthetic replay only; the supplied log is not a full-rate ADC capture.
  }
  {
    Rig r; r.hold(0); assert(!r.p.present);
    for(int cycle=0;cycle<30;cycle++) {
      r.hold(1500); assert(r.p.present);
      int32_t frozen = r.p.zero;
      r.hold(1500,100); assert(r.p.zero==frozen);
      r.hold(cycle%2 ? -50 : 50);
      assert(!r.p.present); assert(r.p.weight==0);
    }
  }
  {
    Rig r(-1000); r.hold(0); r.hold(800); assert(r.p.present);
    r.hold(60); assert(!r.p.present); assert(r.p.weight==0);
  }
  {
    Rig r; r.hold(1500); r.hold(300);
    assert(r.p.present); assert(r.p.zero==r.ref); // partial removal
    r.hold(0); assert(!r.p.present);
  }
  {
    Rig r; r.hold(0);
    for(int i=0;i<30;i++) { r.sample(2000); r.sample(0); r.sample(0); }
    assert(!r.p.present); // isolated spikes rejected by median
    r.hold(1500); r.sample(0); r.sample(0); r.hold(1500);
    assert(r.p.present); // brief lifting is not a completed removal
  }
  {
    Rig r; r.hold(1500);
    for(int i=0;i<100;i++) r.sample(i%2 ? 100 : -100);
    assert(r.p.present); // noisy apparent empty must not retare
    r.hold(30); assert(!r.p.present);
  }
  {
    Rig r; r.hold(1500);
    for(int dg=1500;dg>=0;dg-=25) r.hold(dg,3);
    r.hold(0); assert(!r.p.present); // slow removal
  }
  {
    Rig r; r.hold(0);
    for(int dg=0;dg<=900;dg+=10) r.hold(dg);
    assert(r.p.present); // slow addition cannot shift zero indefinitely
    assert(PresenceTracker::magnitude((r.p.zero-r.ref)*100/r.scale)<=240);
  }
  {
    Rig r; r.hold(1500); int32_t zero=r.p.zero;
    r.p.interrupt(); r.now+=5000; r.hold(1500);
    assert(r.p.present && r.p.zero==zero); // errors preserve occupied baseline
    r.hold(0,2); r.now+=5000; r.hold(0,2);
    assert(r.p.present); // gaps cannot count as stable dwell
    r.hold(0); assert(!r.p.present);
  }
  {
    Rig r; r.hold(1500); r.hold(-400); assert(!r.p.present);
    assert(r.p.zero==r.ref); // release without silently accepting a large new zero
    r.hold(1500); assert(r.p.present); // re-arm on a new confirmed load
  }
  {
    Rig r; r.hold(800); assert(r.p.present); // boot loaded: saved reference retained
    r.now=0xfffffc00U; r.p.interrupt(); r.hold(0,40); assert(!r.p.present);
  }
  {
    StableSamples w={}; assert(!w.add(12345,0,10));
    for(uint32_t i=1;i<20;i++) assert(!w.add(12345,i*50,10));
    w.clear(); bool ok=false;
    for(uint32_t i=0;i<25;i++) ok=w.add(12345+i%3,i*100,10);
    assert(ok);
    for(uint32_t i=0;i<100;i++) assert(!w.add(i*100,3000+i*100,10));
  }
  puts("PASS: repeated +/- residual cycles, polarity, partial/slow removal, spikes, vibration, drift cap, faults, gaps, boot loaded, millis wrap, stable tare");
}

#include <gtest/gtest.h>
#include "image_staging.h"
#include "devices/peripherals.h"
#include <array>
#include <vector>
#include <algorithm>

namespace {
struct Storage {
  eppg::sim::devices::NorFlash flash{EPPG_STAGE_DATA_OFFSET + EPPG_STAGE_MAX_IMAGE};
  unsigned writes=0, erases=0;
  eppg_stage_io io() { return {this, static_cast<uint32_t>(flash.size()),
    [](void* c,uint32_t o,void* p,size_t n){return static_cast<Storage*>(c)->flash.read(o,static_cast<uint8_t*>(p),n)?0:-1;},
    [](void* c,uint32_t o,const void* p,size_t n){auto& s=*static_cast<Storage*>(c);++s.writes;return s.flash.program(o,static_cast<const uint8_t*>(p),n)?0:-1;},
    [](void* c,uint32_t o,size_t n){auto& s=*static_cast<Storage*>(c);++s.erases;return s.flash.erase(o,n)?0:-1;}}; }
  eppg_image_store store() { eppg_image_store s;auto callbacks=io();eppg_image_store_init(&s,&callbacks);return s; }
};
TEST(ImageStaging, FullMaximumImageUsesOneSectorScratch) {
  Storage disk;auto s=disk.store();ASSERT_LE(sizeof(s),EPPG_STAGE_SECTOR_SIZE+128u);
  ASSERT_TRUE(eppg_image_store_begin(&s,EPPG_STAGE_MAX_IMAGE));
  std::vector<uint8_t> image(EPPG_STAGE_MAX_IMAGE);for(size_t i=0;i<image.size();++i)image[i]=static_cast<uint8_t>((i*31u)^(i>>8));
  for(size_t i=0;i<image.size();i+=173){size_t n=std::min<size_t>(173,image.size()-i);ASSERT_EQ(eppg_image_store_write(&s,i,image.data()+i,n),i+n);}
  ASSERT_TRUE(eppg_image_store_finish(&s));std::vector<uint8_t> actual(image.size());ASSERT_TRUE(eppg_image_store_read(&s,0,actual.data(),actual.size()));EXPECT_EQ(actual,image);
  EXPECT_EQ(eppg_image_store_write(&s,0,image.data(),1),-1);
}
TEST(ImageStaging, CrossSectorRetryPreservesNeighboursAndDuplicateDoesNotWearFlash) {
  Storage disk;auto s=disk.store();ASSERT_TRUE(eppg_image_store_begin(&s,8192));
  std::vector<uint8_t> original(8192,0x55);ASSERT_EQ(eppg_image_store_write(&s,0,original.data(),original.size()),8192);
  auto writes=disk.writes, erases=disk.erases;EXPECT_EQ(eppg_image_store_write(&s,0,original.data(),original.size()),8192);EXPECT_EQ(disk.writes,writes);EXPECT_EQ(disk.erases,erases);
  std::array<uint8_t,8> changed{0xff,0x00,0xaa,0x11,0x22,0x33,0x44,0xff};
  ASSERT_EQ(eppg_image_store_write(&s,4093,changed.data(),changed.size()),8192);std::copy(changed.begin(),changed.end(),original.begin()+4093);
  std::vector<uint8_t> actual(8192);ASSERT_TRUE(eppg_image_store_read(&s,0,actual.data(),actual.size()));EXPECT_EQ(actual,original);EXPECT_EQ(disk.erases,erases+2);
}
TEST(ImageStaging, HolesCannotSealAndBoundsAreChecked) {
  Storage disk;auto s=disk.store();ASSERT_FALSE(eppg_image_store_begin(&s,0));ASSERT_FALSE(eppg_image_store_begin(&s,EPPG_STAGE_MAX_IMAGE+1));ASSERT_TRUE(eppg_image_store_begin(&s,8));
  std::array<uint8_t,8> bytes{1,2,3,4,5,6,7,8};EXPECT_EQ(eppg_image_store_write(&s,4,bytes.data()+4,4),0);EXPECT_FALSE(eppg_image_store_finish(&s));
  EXPECT_EQ(eppg_image_store_write(&s,0,bytes.data(),4),4);EXPECT_FALSE(eppg_image_store_finish(&s));EXPECT_EQ(eppg_image_store_write(&s,4,bytes.data()+4,4),8);EXPECT_TRUE(eppg_image_store_finish(&s));EXPECT_FALSE(eppg_image_store_read(&s,7,bytes.data(),2));
}
TEST(ImageStaging, InterruptedFirstClaimCanRecoverOnlyOnEmptyPartition) {
  for(size_t cut=0;cut<16;++cut){Storage disk;auto s=disk.store();disk.flash.cut_power_after(cut);EXPECT_FALSE(eppg_image_store_begin(&s,100));disk.flash.restore_power();auto restart=disk.store();EXPECT_TRUE(eppg_image_store_begin(&restart,100))<<cut;}
  Storage disk;uint8_t existing=0x12;ASSERT_TRUE(disk.flash.program(500,&existing,1));auto s=disk.store();EXPECT_FALSE(eppg_image_store_begin(&s,100));EXPECT_EQ(disk.erases,0u);EXPECT_EQ(disk.writes,0u);uint8_t actual=0;ASSERT_TRUE(disk.flash.read(500,&actual,1));EXPECT_EQ(actual,existing);
}
TEST(ImageStaging, PowerCutDuringImageWriteRestartsWithoutLosingOwnership) {
  Storage disk;auto s=disk.store();ASSERT_TRUE(eppg_image_store_begin(&s,8192));std::array<uint8_t,16> marker{};ASSERT_TRUE(disk.flash.read(0,marker.data(),marker.size()));
  std::vector<uint8_t> image(8192,0x42);disk.flash.cut_power_after(137);EXPECT_EQ(eppg_image_store_write(&s,0,image.data(),image.size()),-1);disk.flash.restore_power();
  auto restart=disk.store();ASSERT_TRUE(eppg_image_store_begin(&restart,image.size()));EXPECT_EQ(restart.received,0u);ASSERT_EQ(eppg_image_store_write(&restart,0,image.data(),image.size()),image.size());ASSERT_TRUE(eppg_image_store_finish(&restart));std::array<uint8_t,16> after{};ASSERT_TRUE(disk.flash.read(0,after.data(),after.size()));EXPECT_EQ(marker,after);
}
TEST(ImageStaging, PowerCutDuringEraseAndSectorRewriteCanRestartSafely) {
  for(bool rewrite: {false,true}){Storage disk;auto s=disk.store();ASSERT_TRUE(eppg_image_store_begin(&s,8192));std::vector<uint8_t> zero(8192,0);ASSERT_EQ(eppg_image_store_write(&s,0,zero.data(),zero.size()),8192);eppg_image_store_abort(&s);
    if(rewrite){ASSERT_TRUE(eppg_image_store_begin(&s,8192));ASSERT_EQ(eppg_image_store_write(&s,0,zero.data(),zero.size()),8192);}
    disk.flash.cut_power_after(201);if(rewrite){uint8_t ff=0xff;EXPECT_EQ(eppg_image_store_write(&s,10,&ff,1),-1);}else EXPECT_FALSE(eppg_image_store_begin(&s,8192));
    disk.flash.restore_power();auto restart=disk.store();ASSERT_TRUE(eppg_image_store_begin(&restart,8192));std::vector<uint8_t> actual(8192);ASSERT_TRUE(eppg_image_store_read(&restart,0,actual.data(),actual.size()));EXPECT_TRUE(std::all_of(actual.begin(),actual.end(),[](uint8_t b){return b==0xff;}));
  }
}
}
int main(int argc,char**argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

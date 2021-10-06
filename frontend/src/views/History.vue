<template>
  <div>
    <Navbar :title="'알림'" :left_icon="true" :right_text="''" :left_push="'Home'" :right_push="''"/>
    <div class="history-container">

    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'

export default {
  name: 'History',
  components: {
    Navbar,
  },
  data() {
    return {
      data: [
        {
          data: '날짜시간형식',
          time: '10: 30',
          title: '침입자 감지',
          description: 'OO방에서 침입자를 감지했습니다.',
          pic: require('@/assets/images/cam.png')
        },
        {
          data: '날짜시간형식',
          time: '9: 30',
          title: '침입자 감지',
          description: 'OO방에서 침입자를 감지했습니다.',
          pic: require('@/assets/images/cam.png')
        },
        {
          data: '날짜시간형식',
          time: '11: 30',
          title: '침입자 감지',
          description: 'OO방에서 침입자를 감지했습니다.',
          pic: ''
        }
      ]
    }
  }
}
</script>

<style lang="scss" scoped>
  @import "./History.scss";
</style>
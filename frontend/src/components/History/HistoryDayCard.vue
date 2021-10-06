<template>
  <div>
    <div class="date-bar font-400">{{ date }} 2021년 10월 10일</div>
    <div class="card-container">
      <HistoryCard/>
    </div>
  </div>
</template>

<script>
import HistoryCard from '@/components/History/HistoryCard.vue'

export default {
  name: 'HistoryDayCard',
  components: {
    HistoryCard
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
  @import './HistoryDayCard.scss';
</style>
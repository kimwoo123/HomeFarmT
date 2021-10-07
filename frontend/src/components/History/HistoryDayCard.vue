<template>
  <div>
    <div class="date-bar font-400">{{ dateArr[0] + '년 ' + dateArr[1] + '월 ' + dateArr[2] + '일 ' }}</div>
    <div class="card-container" v-for="(history, index) in dailyHistory" :key="index">
      <HistoryCard :history="history"/>
    </div>
  </div>
</template>

<script>
import HistoryCard from '@/components/History/HistoryCard.vue'

export default {
  name: 'HistoryDayCard',
  props: ['history', 'date'],
  components: {
    HistoryCard
  },
  data() {
    return {
      dailyHistory: [],
      dateArr: this.date.split('T')[0].split('-')
    }
  },
  created() {
    this.history.forEach(element => {
      if (element.event_time.substring(0, 10) === this.date) {
        this.dailyHistory.push(element)
      }
    });
  }
}
</script>

<style lang="scss" scoped>
  @import './HistoryDayCard.scss';
</style>
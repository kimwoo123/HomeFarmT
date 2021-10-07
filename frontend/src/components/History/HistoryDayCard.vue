<template>
  <div>
    <div class="date-bar font-400">{{ date }}</div>
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
      dailyHistory:[],
    }
  },
  created() {
    this.history.forEach(element => {
      if (element.event_time === this.date) {
        this.dailyHistory.push(element)
      }
    });
  }
}
</script>

<style lang="scss" scoped>
  @import './HistoryDayCard.scss';
</style>
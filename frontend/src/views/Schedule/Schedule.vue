<template>
  <div>
    <Navbar :title="'10월 스케줄'" :left_icon="true" :right_text="'추가'"/>
    <div class="schedule-container">
      <ScheduleCalendar/>
      <ScheduleCard/>
    </div>
    <!-- <SchedulePlan/> -->
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import ScheduleCalendar from '@/components/Schedule/ScheduleCalendar.vue'
// import SchedulePlan from '@/components/Schedule/SchedulePlan.vue'
import ScheduleCard from '@/components/Schedule/ScheduleCard.vue'

export default {
  name: 'Schedule',
  components: {
    Navbar,
    ScheduleCalendar,
    // SchedulePlan,
    ScheduleCard
  },
  data() {
    return {
      today: '',
      todayWeek: '',
      targetWeeok: '',
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './Schedule.scss';
</style>
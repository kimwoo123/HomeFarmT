<template>
  <div>

    <div class="calendar">
      <div class="day">
        9dad
      </div>
    </div>


    schedule
    <div class="schedule">
      <div v-for="time in 25" :key=time.id>

        <div v-if="time <= 10" class="time-box">
          0{{ time-1 }}:00
        </div>
        <div v-else class="time-box">
          {{ time-1 }}:00
        </div>
        
      </div>
    </div>

  </div>
</template>


<script>
export default {
  name: 'Schedule',
  data() {
    return {
      today: {
        year: 2021,
        month: 9,
        day:14
      },
      schedules: [
        {
          start: '09:00',
          end: '12:00',
          content: 'carrot 물 주기'
        },
        {
          start: '14:00',
          end: '17:00',
          content: 'carrot asdaadad'
        },
      ],
    }
  }
}
</script>

<style lang="scss" scoped>
  @import url('./Schedule.scss');
</style>
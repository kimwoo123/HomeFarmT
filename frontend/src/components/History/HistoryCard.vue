<template>
  <div class="history-card-container">
    <div>
      <span class="font-400 time">{{ history.event_time.split('T')[1] }}</span>
    </div>

    <div class="circle"></div>

    <div class="history-card shadows">
      
      <div v-if="pic && !picBig" @click="picBig=true" class="small-card">
        <div>
          <p class="title">{{ history.event_title }}</p>
          <p class="description">{{ history.event_desc }}</p>   
        </div>
        <img :src="`${ history.event_img }`" alt="" class="small-pic">
      </div>
      
      <div v-else-if="pic && picBig" @click="picBig=false" class="big-card">
        <div>
          <p class="title">{{ history.event_title }}</p>
          <p class="description">{{ history.event_desc }}</p>  
        </div>
        <img :src="`${ history.event_img }`" alt="" class="big-pic">
      </div>

      <div v-else class="small-card">
        <div>
          <p class="title">{{ history.event_title }}</p>
          <p class="description">{{ history.event_desc }}</p>  
        </div>
      </div>

    </div>
  </div>
</template>

<script>
export default {
  name: 'HistoryCard',
  props: ['history'],
  data() {
    return {
      picBig: false,
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './HistoryCard.scss';
</style>
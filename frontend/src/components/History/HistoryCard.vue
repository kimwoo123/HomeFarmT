<template>
  <div class="history-card-container">
    <div>
      <span class="font-400 time">{{ time }}</span>
    </div>

    <div class="circle"></div>

    <div class="history-card shadows">
      
      <div v-if="pic && !picBig" @click="picBig=true" class="small-card">
        <div>
          <p class="title">{{ title }}</p>
          <p class="description">{{ description }}</p>   
        </div>
        <img :src="`${ pic }`" alt="" class="small-pic">
      </div>
      
      <div v-else-if="pic && picBig" @click="picBig=false" class="big-card">
        <div>
          <p class="title">{{ title }}</p>
          <p class="description">{{ description }}</p>
        </div>
        <img :src="`${ pic }`" alt="" class="big-pic">
      </div>

      <div v-else class="small-card">
        <div>
          <p class="title">{{ title }}</p>
          <p class="description">{{ description }}</p>
        </div>
      </div>

    </div>
  </div>
</template>

<script>
export default {
  name: 'HistoryCard',
  data() {
    return {
      data: '날짜시간형식',
      time: '10: 30',
      title: '침입자 감지',
      description: 'OO방에서 침입자를 감지했습니다.',
      pic: require('@/assets/images/cam.png'),
      picBig: false,
    }
  }

}
</script>

<style lang="scss" scoped>
  @import './HistoryCard.scss';
</style>
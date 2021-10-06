<template>
  <div class="patrol-path shadows">
    
    <div class="text-content">
      <div>
        <span class="target-path">{{ pathName }}</span>
        <span>로 감시를 실행합니다.</span>
      </div>
      <span v-if="!status" class="status font-300">상태 이상없음</span>
      <span v-else class="status font-300">상태 이상 발생</span>
    </div>
    <SwitchBtn class="switch-align" @is-checked="changePatrolOn" :defaultValue="parseInt(patrolOn)" :checkboxId="1"/>
  </div>
</template>

<script>
import SwitchBtn from '@/components/common/SwitchBtn.vue'

export default {
  name: 'PatrolPath',
  components: {
    SwitchBtn
  },
  data() {
    return {
      path: localStorage.getItem('currentPath') || '1',
      patrolOn: localStorage.getItem('patrolOn') || '0',
      status: 0
    }
  },
  computed: {
    pathName() {
      const name = 'path' + this.path
      return localStorage.getItem(name)
    }
  },
  methods: {
    btn_patrol_on() {
      console.log('btn_patrol_on');
      let data = 1;
      this.$socket.emit('PatrolOnToServer', data);
    },
    btn_patrol_off() {
      console.log('btn_patrol_off');
      let data = 0;
      this.$socket.emit('PatrolOffToServer', data);
    },
    changePatrolOn(isChecked) {
      this.patrolOn = isChecked
      if (isChecked) {
        localStorage.setItem('patrolOn', '1');
        this.btn_patrol_on()
      } else {
        localStorage.setItem('patrolOn', '0');
        this.btn_patrol_off()
      }
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './PatrolPath.scss';
</style>
import Vue from 'vue'
import VueRouter from 'vue-router'
import Signup from '../views/user/Signup.vue'
import Login from '../views/user/Login.vue'
import Home from '../views/Home.vue'
import IoT from '../views/IoT.vue'
import Schedule from '../views/Schedule.vue'
import ScheduleNew from '../views/ScheduleNew.vue'
import Patrol from '../views/Patrol.vue'
import PatrolSetting from '../views/PatrolSetting.vue'
import Control from '../views/Control.vue'
import Test from '../views/Test.vue'
import SimulationMap from '../components/common/SimulationMap.vue'

Vue.use(VueRouter)

const routes = [
  {
    path: '*',
    redirect: '/'
  },
  {
    path: '/',
    redirect: Home,
  },
  {
    path: '/signup',
    name: 'Signup',
    component: Signup
  },
  {
    path: '/login',
    name: 'Login',
    component: Login
  },
  {
    path: '/home',
    name: 'Home',
    component: Home
  },
  // iot
  {
    path: '/iot',
    name: 'IoT',
    component: IoT
  },
  // schedule
  {
    path: '/schedule',
    name: 'Schedule',
    component: Schedule
  },
  {
    path: '/schedule/new',
    name: 'ScheduleNew',
    component: ScheduleNew
  },
  // patrol
  {
    path: '/patrol',
    name: 'Patrol',
    component: Patrol
  },
  {
    path: '/patrol/setting',
    name: 'PatrolSetting',
    component: PatrolSetting
  },
  // control 
  {
    path: '/control',
    name: 'Control',
    component: Control
  },
  {
    path: '/map',
    name: 'SimulationMap',
    component: SimulationMap
  },
  {
    path: '/test',
    name: 'Test',
    component: Test
  },
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

export default router

const { Schedule } = require("../../models/index")
const { User } = require("../../models/index")
const { GraphQLScalarType, Kind } = require('graphql');

module.exports = {
  Query: {
    allSchedule: async () =>{
      if(context.user) {
        console.log(context.user)
      }
      const getSchedules = await Schedule.findAll();
      return getSchedules;
    },
    getSchedule: async (_, args, context) => {
      if (!context) {
        throw new Error('로그인이 필요합니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      const userid = user.dataValues.userid
      const userSchedule = await Schedule.findAll({ where: { userid: userid }});
      let scheduleList = []
      for (let schedule of userSchedule) {
        scheduleList.push(schedule.dataValues)
      }
      // console.log(scheduleList)
      // const result = {
      //   schedule_time: scheduleList
      // }
      return scheduleList
    }
  },
  Mutation: {
    createSchedule: async (_, { schedule_time, schedule_title, schedule_desc, schedule_status }, context) => {
      console.log('----------------------------')
      if (!context) {
        throw new Error('인증된 유저가 아닙니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      const userid = user.dataValues.userid
      const newSchedule = await Schedule.create({ 
        schedule_time: schedule_time,
        schedule_title: schedule_title,
        schedule_desc: schedule_desc,
        schedule_status: schedule_status,
        userid: userid
       })
      return newSchedule
    },
    updateSchedule: async (_, { scheduleid, schedule_time }) => {
      await Schedule.update({ schedule_time: schedule_time }, {
        where: {
          scheduleid: scheduleid
        }
      })
    },
    deleteSchedule: async (_, { scheduleid }) => {
      await Schedule.destroy({ where: { scheduleid }})
    }
  }
};

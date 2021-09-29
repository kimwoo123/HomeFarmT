const { Schedule } = require("../../models/index")

module.exports = {
  Query: {
    allSchedule: async () =>{
      const getSchedules = await Schedule.findAll();
      return getSchedules;
    },
    findSchedule: async (_, { schedule_title }) => {
      const oneSchedule = await Schedule.findOne({ where: {schedule_title: schedule_title}});
      return oneSchedule
    }
  },
  Mutation: {
    createSchedule: async (_, { schedule_title, schedule_dec }) => {
      const newSchedule = await Schedule.create({ schedule_title, schedule_dec })
      return newSchedule
    },
    updateSchedule: async (_, { scheduleid, schedule_title, schedule_dec }) => {
      await Schedule.update({ schedule_title: schedule_title, schedule_dec: schedule_dec }, {
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
